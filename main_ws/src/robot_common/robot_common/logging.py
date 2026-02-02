# from __future__ import annotations

"""
I added this module to provide a unified logging mechanism with deduplication
and to simplify logging calls across the codebase.
Some functions were refactored to use this module.
Others will be refactored in the future.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class LogLevel(Enum):
    INFO = auto()
    WARN = auto()
    ERROR = auto()
    DEBUG = auto()


@dataclass
class LogEvent:
    message: str
    source: str
    level: LogLevel
    counter: int = 0
    exception: Optional[Exception] = None

    def same_identity(self, other: "LogEvent") -> bool:
        return (
            self.level == other.level and
            self.source == other.source and
            self.message == other.message
        )

#--------------------------------------------------------------------------------
def log_event(
    logger,
    event: LogEvent,
    last_event: Optional[LogEvent] = None,
) -> LogEvent:
    """
    Unified logging with dedup multiplier. Use ONE shared last_event variable.

    Behavior:
    - If event is same as last_event: increment counter and print "\r x N"
    - Else: newline, emit ROS log line, reset counter to 0
    """
    if last_event is not None and last_event.same_identity(event):
        event.counter = last_event.counter + 1
        print(f"\r x {event.counter}\t", end="", flush=True)
        return event

    # New event: print newline so the next log starts cleanly
    print("\n")

    # Emit via ROS logger
    if event.level == LogLevel.ERROR:
        if event.exception is not None:
            logger.error(
                f"Error in {event.source}: {str(event.exception)}. {event.message}".strip()
            )
        else:
            logger.error(event.message)
    elif event.level == LogLevel.WARN:
        logger.warn(f"{event.source}: {event.message}".strip() if event.source else event.message)
    elif event.level == LogLevel.DEBUG:
        logger.debug(f"{event.source}: {event.message}".strip() if event.source else event.message)
    else:
        logger.info(f"{event.source}: {event.message}".strip() if event.source else event.message)

    event.counter = 0
    return event

#--------------------------------------------------------------------------------
# Convenience wrappers 
def log_info(logger, message: str, source: str = "", last_event: Optional[LogEvent] = None) -> LogEvent:
    return log_event(logger, LogEvent(message=message, source=source, level=LogLevel.INFO), last_event)

#--------------------------------------------------------------------------------
def log_error(logger, error: Exception, source: str, message: str = "", last_event: Optional[LogEvent] = None) -> LogEvent:
    return log_event(logger, LogEvent(message=message, source=source, level=LogLevel.ERROR, exception=error), last_event)
