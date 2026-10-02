from datetime import datetime, timezone
import json
import logging
from pathlib import Path


KNOWN_STATES = {
    "IDLE",
    "TRACKING",
    "HOVERING",
    "DOCKED",
    "DOCKING_INIT",
    "DOCKING_APPROACH",
    "ABORT",
    "EMERGENCY_LAND",
}


class UniversalLog:
    def __init__(self, info, print_to_terminal=False):
        if not isinstance(info, dict):
            raise TypeError("Each log entry must be a JSON object")

        state = info.get("State", "UNKNOWN")
        timestamp = info.get("Timestamp") or datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        )
        details = info.get("Details", {})
        if not isinstance(details, dict):
            raise TypeError("'Details' must be a JSON object")

        details_json = json.dumps(details, separators=(",", ":"))
        message = f"[{timestamp}] {state}: {details_json}"
        level = logging.INFO if state in KNOWN_STATES else logging.ERROR

        logger = logging.getLogger("universal_log")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        formatter = logging.Formatter("%(message)s")
        if not any(isinstance(handler, logging.FileHandler) for handler in logger.handlers):
            run_id = datetime.now().strftime("%Y%m%d_%f")
            file_handler = logging.FileHandler(
                Path(__file__).with_name(f"logged_states_{run_id}.log"),
                mode="x",
                encoding="utf-8",
            )
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

        console_handler = getattr(logger, "_universal_console_handler", None)
        if print_to_terminal and console_handler is None:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            logger._universal_console_handler = console_handler
            logger.addHandler(console_handler)
        elif not print_to_terminal and console_handler is not None:
            logger.removeHandler(console_handler)
            console_handler.close()
            logger._universal_console_handler = None

        logger.log(level, message)


"""def main(): 
    print_to_terminal = False
    test_file = Path(__file__).with_name("test_states.json")
    with test_file.open(encoding="utf-8") as json_file:
        test_entries = json.load(json_file)

    for entry in test_entries:
        UniversalLog(entry, print_to_terminal=print_to_terminal)


if __name__ == "__main__":
    main()
"""