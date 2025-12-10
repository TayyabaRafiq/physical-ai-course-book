"""
Logging Configuration for VLA Integration
Provides structured JSON logging with rich console output
"""

import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional

import structlog
from rich.console import Console
from rich.logging import RichHandler

# Global console for rich output
console = Console()


def setup_logging(
    log_level: str = "INFO",
    log_dir: Optional[Path] = None,
    enable_json_logging: bool = True,
    service_name: str = "vla-integration"
) -> None:
    """
    Configure structured logging for the VLA pipeline.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_dir: Directory for log files (None = logs to stdout only)
        enable_json_logging: If True, write JSON logs to file
        service_name: Service identifier for log entries
    """
    # Convert log level string to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Configure standard library logging
    logging.basicConfig(
        level=numeric_level,
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(console=console, rich_tracebacks=True)]
    )

    # Configure structlog processors
    processors = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.add_log_level,
        structlog.stdlib.add_logger_name,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
    ]

    # Add JSON rendering for file output
    if enable_json_logging and log_dir:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / f"{service_name}.log"

        # File handler with JSON formatting
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        
        # Add JSON processor for file logs
        processors.append(structlog.processors.JSONRenderer())
        
        # Add file handler to root logger
        logging.getLogger().addHandler(file_handler)
    else:
        # Console output with nice formatting
        processors.append(structlog.dev.ConsoleRenderer())

    # Configure structlog
    structlog.configure(
        processors=processors,
        wrapper_class=structlog.stdlib.BoundLogger,
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )


def get_logger(name: str) -> structlog.stdlib.BoundLogger:
    """
    Get a structured logger instance.

    Args:
        name: Logger name (typically __name__)

    Returns:
        Configured structlog logger
    """
    return structlog.get_logger(name)


def log_pipeline_event(
    logger: structlog.stdlib.BoundLogger,
    event_type: str,
    stage: str,
    **context: Any
) -> None:
    """
    Log a pipeline event with structured context.

    Args:
        logger: Structlog logger instance
        event_type: Type of event (e.g., "transcription_start", "plan_generated")
        stage: Pipeline stage (e.g., "voice", "cognition", "execution")
        **context: Additional context fields
    """
    logger.info(
        f"Pipeline event: {event_type}",
        event_type=event_type,
        stage=stage,
        **context
    )


def log_error_with_context(
    logger: structlog.stdlib.BoundLogger,
    error: Exception,
    stage: str,
    **context: Any
) -> None:
    """
    Log an error with full context and stack trace.

    Args:
        logger: Structlog logger instance
        error: Exception that occurred
        stage: Pipeline stage where error occurred
        **context: Additional context fields
    """
    logger.error(
        f"Error in {stage}: {str(error)}",
        error_type=type(error).__name__,
        error_message=str(error),
        stage=stage,
        exc_info=True,
        **context
    )


# Example usage
if __name__ == "__main__":
    # Setup logging
    setup_logging(
        log_level="DEBUG",
        log_dir=Path("logs"),
        enable_json_logging=True,
        service_name="vla-test"
    )

    # Get logger
    logger = get_logger(__name__)

    # Test logging
    logger.info("VLA logging system initialized", component="logging_config")
    
    log_pipeline_event(
        logger,
        event_type="test_event",
        stage="voice",
        confidence=0.95,
        duration_ms=150
    )

    try:
        raise ValueError("Test error")
    except Exception as e:
        log_error_with_context(logger, e, "test_stage", test_context="example")
