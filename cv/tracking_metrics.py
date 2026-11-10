"""Truth-based identity measurements plus separate emitted-lifecycle audits."""

from collections import Counter
import numpy as np


def fraction(success, count):
    return {"successes": success, "opportunities": count, "rate": success/count if count else None}


def evaluate_sequence(frames, updates, *, max_gap_ns=500_000_000, corner_gate_px=8.):
    if len(frames) != len(updates):
        raise ValueError("One update required per truth frame")
    if type(max_gap_ns) is not int or max_gap_ns <= 0 or not np.isfinite(corner_gate_px) or corner_gate_px <= 0:
        raise ValueError("Gap and corner gate must be positive")
    history, prior_tracks, confirmations = {}, {}, {}
    pairs = {key: [0, 0] for key in ("identity_continuity", "short_gap_reacquisition", "new_id_after_expiry", "ambiguity_refusal")}
    tp = fp = fn = switches = 0
    expiry_violations = retention_violations = 0
    expected_expirations, expiry_errors = 0, []
    delays, evaluations, stage_times = [], [], {}
    for frame, update in zip(frames, updates):
        now = frame["timestamp_ns"]
        if update.frame_index != frame["frame_index"] or update.update_timestamp_ns != now:
            raise ValueError("Truth/update identity or time mismatch")
        truth = [obj for obj in frame["objects"] if obj["observable"]]
        counts = Counter(obj["object_id"] for obj in truth)
        candidates = []
        for i, obj in enumerate(truth):
            for j, detection in enumerate(update.detections):
                if obj["object_id"] == detection.object_id:
                    error = np.sqrt(np.mean(np.sum((np.array(obj["corners_px"])-detection.corners_px)**2, axis=1)))
                    if error <= corner_gate_px:
                        candidates.append((float(error), i, j))
        matched, used = {}, set()
        for _, i, j in sorted(candidates):
            if i not in matched and j not in used:
                matched[i] = update.detections[j]
                used.add(j)
        tp += len(matched)
        fp += len(update.detections)-len(matched)
        fn += len(truth)-len(matched)
        events = []
        for i, obj in enumerate(truth):
            detection = matched.get(i)
            identity = None if detection is None else detection.tracking_id
            if counts[obj["object_id"]] > 1:
                pair = pairs["ambiguity_refusal"]
                pair[1] += 1
                ok = detection is not None and identity is None and detection.association_reason == "ambiguous_object_id"
                pair[0] += int(ok)
                events.append({"entity_id": obj["entity_id"], "check": "ambiguity_refusal", "passed": ok})
                continue
            previous = history.get(obj["entity_id"])
            segment_start = now
            if previous:
                old_time, old_id, segment_start = previous
                gap = now-old_time
                category = ("new_id_after_expiry" if gap >= max_gap_ns else
                            "short_gap_reacquisition" if gap > 100_000_000 else "identity_continuity")
                pair = pairs[category]
                pair[1] += 1
                ok = identity is not None and old_id is not None and (
                    identity != old_id if category == "new_id_after_expiry" else identity == old_id)
                pair[0] += int(ok)
                if gap < max_gap_ns and identity is not None and old_id is not None and identity != old_id:
                    switches += 1
                if gap >= max_gap_ns:
                    segment_start = now
                events.append({"entity_id": obj["entity_id"], "check": category, "passed": ok})
            # Truth visibility, not detector success, defines opportunities.
            history[obj["entity_id"]] = (now, identity, segment_start)
            key = (obj["entity_id"], segment_start)
            confirmations.setdefault(key, None)
            track = next((t for t in update.tracks if t.tracking_id == identity), None)
            if track is not None and track.status == "confirmed" and confirmations[key] is None:
                confirmations[key] = now-segment_start
                delays.append((now-segment_start)/1e6)
        current = {t.tracking_id: t for t in update.tracks}
        for identity, old in prior_tracks.items():
            if old.status == "expired":
                continue
            new = current.get(identity)
            due = now-old.last_observed_timestamp_ns >= max_gap_ns
            expected_expirations += int(due)
            if due and (new is None or new.status != "expired"):
                expiry_violations += 1
            if not due and (new is None or new.status == "expired"):
                expiry_violations += 1
            if new is not None and new.estimate_kind == "retained":
                if (new.last_observed_timestamp_ns != old.last_observed_timestamp_ns
                        or new.estimate_timestamp_ns != old.estimate_timestamp_ns or new.pose != old.pose):
                    retention_violations += 1
        for track in update.tracks:
            if track.status == "expired":
                expiry_errors.append((now-track.last_observed_timestamp_ns-max_gap_ns)/1e6)
            if track.status == "expired" and now-track.last_observed_timestamp_ns < max_gap_ns:
                expiry_violations += 1
        prior_tracks = current
        for stage, duration in update.timings_ns:
            stage_times.setdefault(stage, []).append(duration/1e6)
        evaluations.append({"frame_index": frame["frame_index"], "timestamp_ns": now,
                            "tp": len(matched), "fp": len(update.detections)-len(matched),
                            "fn": len(truth)-len(matched), "identity_checks": events})
    result = {"frames": len(frames), "tp": tp, "fp": fp, "fn": fn,
              "detection_rate": tp/(tp+fn) if tp+fn else None,
              **{name: fraction(*counts) for name, counts in pairs.items()},
              "unexpected_id_changes": switches,
              "confirmation": {"segments": len(confirmations), "confirmed": len(delays),
                               "unconfirmed": sum(v is None for v in confirmations.values()),
                               "p50_delay_ms": float(np.percentile(delays, 50)) if delays else None,
                               "p95_delay_ms": float(np.percentile(delays, 95)) if delays else None},
              "emitted_lifecycle_audit": {"expiry_violations": expiry_violations,
                                          "expected_expirations": expected_expirations,
                                          "expiry_events": len(expiry_errors),
                                          "p95_absolute_expiry_timing_error_ms": float(np.percentile(np.abs(expiry_errors), 95)) if expiry_errors else None,
                                          "retention_timestamp_or_pose_violations": retention_violations},
              "latency": {stage: {"count": len(values), **{f"p{p}_ms": float(np.percentile(values, p))
                            for p in (50, 95, 99)}} for stage, values in stage_times.items()}}
    return result, evaluations
