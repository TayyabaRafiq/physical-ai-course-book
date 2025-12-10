"""
VLA Command Line Interface

Provides interactive CLI for running the VLA pipeline.
"""

import asyncio
import sys
from pathlib import Path

import click

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.vla_core.pipeline.vla_pipeline import VlaPipeline
from src.vla_core.utils.config import load_config
from src.vla_core.utils.errors import VlaError
from src.vla_core.utils.logging_config import setup_logging


@click.group()
@click.option("--log-level", default="INFO", help="Logging level")
@click.option("--config-file", type=click.Path(exists=True), help="Config file path")
@click.pass_context
def cli(ctx, log_level, config_file):
    """VLA Integration - Voice-Language-Action Pipeline for Robot Control."""
    # Setup logging
    setup_logging(log_level=log_level, enable_json=False)

    # Load configuration
    config_path = Path(config_file) if config_file else None
    try:
        load_config(env_file=config_path, validate=False)
    except Exception as e:
        click.echo(f"Warning: Failed to load config: {e}", err=True)

    ctx.ensure_object(dict)


@cli.command()
@click.option(
    "--timeout",
    type=float,
    default=10.0,
    help="Voice input timeout in seconds",
)
@click.option(
    "--loop/--no-loop",
    default=False,
    help="Run in continuous loop",
)
def run(timeout, loop):
    """Run the VLA pipeline interactively."""
    click.echo("=" * 60)
    click.echo("VLA Integration - Voice Control System")
    click.echo("=" * 60)
    click.echo()

    async def run_pipeline():
        """Run pipeline async."""
        # Create pipeline
        pipeline = VlaPipeline()

        try:
            # Initialize
            click.echo("Initializing pipeline (loading Whisper model)...")
            await pipeline.initialize()

            click.echo(f"✓ Pipeline ready!")
            click.echo()

            # Run loop
            iteration = 0
            while True:
                iteration += 1

                if loop:
                    click.echo(f"\n--- Iteration {iteration} ---")

                click.echo(f"🎤 Speak your command (timeout: {timeout}s)...")
                click.echo("   Examples:")
                click.echo("     - 'Pick up the red block'")
                click.echo("     - 'Move forward 2 meters'")
                click.echo("     - 'Stop'")
                click.echo()

                try:
                    # Process command with progress display
                    click.echo("\n📋 Processing voice command...")

                    # Listen
                    click.echo("🎤 Listening...")
                    command = await pipeline.voice_interface.listen(timeout)
                    await pipeline.state_manager.store_voice_command(command)
                    click.secho(f"✓ Heard: '{command.transcribed_text}'", fg="cyan")
                    click.echo(f"  Confidence: {command.confidence:.2%}")

                    # Parse intent
                    click.echo("\n🧠 Parsing intent...")
                    intent = await pipeline.intent_parser.parse(command)
                    await pipeline.state_manager.store_intent(intent)
                    click.secho(f"✓ Intent: {intent.action_type.value}", fg="cyan")
                    if intent.target_objects:
                        click.echo(f"  Objects: {', '.join(obj.name for obj in intent.target_objects)}")

                    # Generate plan
                    click.echo("\n🗺️  Generating action plan...")
                    plan = await pipeline.action_planner.generate_plan(intent)
                    await pipeline.state_manager.store_plan(plan)
                    click.secho(f"✓ Plan generated: {len(plan.steps)} steps", fg="cyan")

                    # Display plan preview
                    click.echo("\n" + "─" * 60)
                    click.secho("PROPOSED ACTION PLAN", fg="yellow", bold=True)
                    click.echo("─" * 60)
                    for i, step in enumerate(plan.steps):
                        step_icon = {
                            "navigate": "🚶",
                            "pick": "🤏",
                            "place": "📦",
                            "inspect": "👁️",
                            "wait": "⏸️",
                            "stop": "🛑"
                        }.get(step.action_type.value, "▶️")

                        click.echo(f"{step_icon}  Step {i+1}: {step.action_type.value.upper()}")
                        click.echo(f"    Action: {step.ros_action_name}")
                        click.echo(f"    Timeout: {step.timeout}s")

                        # Show key parameters
                        if step.action_type.value == "navigate":
                            target = step.goal_message.get("target_pose", {})
                            if target:
                                click.echo(f"    Target: x={target.get('x', 0):.2f}, y={target.get('y', 0):.2f}")
                        elif step.action_type.value == "pick":
                            obj_id = step.goal_message.get("object_id", "unknown")
                            grasp = step.goal_message.get("grasp_type", "top")
                            click.echo(f"    Object: {obj_id}, Grasp: {grasp}")
                        elif step.action_type.value == "place":
                            pose = step.goal_message.get("placement_pose", {})
                            if pose:
                                click.echo(f"    Position: x={pose.get('x', 0):.2f}, y={pose.get('y', 0):.2f}, z={pose.get('z', 0):.2f}")

                    click.echo("\n" + "─" * 60)
                    click.echo(f"Estimated duration: {plan.estimated_duration:.1f}s")
                    click.echo(f"Preconditions: {', '.join(plan.preconditions[:3])}")
                    if len(plan.preconditions) > 3:
                        click.echo(f"               ... and {len(plan.preconditions) - 3} more")
                    click.echo("─" * 60)

                    # Validate
                    click.echo("\n✓ Validating plan...")
                    is_valid, errors = await pipeline.plan_validator.validate(plan)

                    if not is_valid:
                        click.secho(f"\n✗ Validation failed:", fg="red", bold=True)
                        for error in errors[:5]:  # Show first 5 errors
                            click.echo(f"  • {error}")
                        if len(errors) > 5:
                            click.echo(f"  ... and {len(errors) - 5} more errors")
                        continue

                    click.secho("✓ Plan validated", fg="green")

                    # Execute
                    click.echo("\n🤖 Executing plan...")
                    click.echo("─" * 60)

                    execution_state = await pipeline.execution_monitor.execute_plan(plan)

                    # Get execution log
                    log = pipeline.execution_monitor.get_current_log()
                    if log:
                        log.voice_command_text = command.transcribed_text
                        log.parsed_intent_summary = (
                            f"{intent.action_type.value}: "
                            f"{', '.join(obj.name for obj in intent.target_objects)}"
                        )

                    # Display results
                    click.echo()
                    click.echo("=" * 60)
                    if execution_state.status.value == "completed":
                        click.secho("✓ EXECUTION COMPLETE", fg="green", bold=True)
                    elif execution_state.status.value == "failed":
                        click.secho("✗ EXECUTION FAILED", fg="red", bold=True)
                    else:
                        click.secho(f"⚠ EXECUTION {execution_state.status.value.upper()}", fg="yellow", bold=True)
                    click.echo("=" * 60)

                    click.echo(f"Command:  {log.voice_command_text if log else command.transcribed_text}")
                    click.echo(f"Intent:   {log.parsed_intent_summary if log else intent.action_type.value}")
                    click.echo(f"Steps:    {len(plan.steps)}")
                    click.echo(f"Completed: {len(execution_state.completed_steps)}/{len(plan.steps)}")
                    click.echo(f"Status:   {execution_state.status.value.upper()}")

                    if log:
                        click.echo(f"Duration: {log.total_duration:.2f}s")
                        click.echo(f"Events:   {len(log.execution_trace)}")

                    # Show errors if any
                    if execution_state.errors:
                        click.echo("\nErrors:")
                        for error in execution_state.errors[:3]:
                            click.secho(f"  • Step {error.step_index}: {error.message}", fg="red")
                        if len(execution_state.errors) > 3:
                            click.echo(f"  ... and {len(execution_state.errors) - 3} more errors")

                    click.echo()

                except KeyboardInterrupt:
                    click.echo("\n\n⚠️  Interrupted by user")
                    break

                except VlaError as e:
                    click.secho(f"\n✗ Error: {e}", fg="red")
                    if not loop:
                        break

                except Exception as e:
                    click.secho(f"\n✗ Unexpected error: {e}", fg="red")
                    if not loop:
                        break

                # Exit if not looping
                if not loop:
                    break

                # Ask to continue
                if loop:
                    click.echo()
                    if not click.confirm("Continue with next command?", default=True):
                        break

        except Exception as e:
            click.secho(f"✗ Pipeline error: {e}", fg="red", err=True)
            return 1

        finally:
            # Cleanup
            await pipeline.emergency_stop()
            click.echo("\n👋 Pipeline stopped")

        return 0

    # Run async
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    exit_code = asyncio.run(run_pipeline())
    sys.exit(exit_code)


@cli.command()
def devices():
    """List available audio input devices."""
    from src.vla_core.voice.audio_capture import AudioCapture

    click.echo("Available Audio Input Devices:")
    click.echo("=" * 60)

    capture = AudioCapture()
    devices = capture.list_devices()

    for device in devices:
        click.echo(
            f"[{device['index']:2d}] {device['name']:<40} {device['sample_rate']}Hz"
        )

    click.echo()
    click.echo(f"Total: {len(devices)} devices")
    click.echo()
    click.echo("To use a specific device, set AUDIO_DEVICE_INDEX in .env")


@cli.command()
def version():
    """Show version information."""
    from src.vla_core import __version__

    click.echo(f"VLA Integration v{__version__}")
    click.echo("Voice-Language-Action Pipeline for Robot Control")


@cli.command()
@click.option(
    "--check-api/--no-check-api",
    default=False,
    help="Check OpenAI API connectivity",
)
def validate(check_api):
    """Validate configuration and dependencies."""
    click.echo("Validating configuration...")
    click.echo()

    try:
        # Load config with validation
        config = load_config(validate=True)

        click.secho("✓ Configuration loaded successfully", fg="green")
        click.echo(f"  Whisper Model: {config.whisper_model}")
        click.echo(f"  OpenAI Model: {config.openai_model}")
        click.echo(f"  Log Level: {config.log_level}")
        click.echo(f"  ROS Domain: {config.ros_domain_id}")
        click.echo()

        # Check Whisper
        click.echo("Checking Whisper...")
        try:
            import whisper

            click.secho("✓ Whisper installed", fg="green")
        except ImportError:
            click.secho("✗ Whisper not installed", fg="red")

        # Check OpenAI API
        if check_api:
            click.echo("\nChecking OpenAI API...")
            try:
                from openai import OpenAI

                client = OpenAI(api_key=config.openai_api_key)
                client.models.list()
                click.secho("✓ OpenAI API accessible", fg="green")
            except Exception as e:
                click.secho(f"✗ OpenAI API error: {e}", fg="red")

        click.echo()
        click.secho("✓ Validation complete", fg="green", bold=True)

    except Exception as e:
        click.secho(f"✗ Validation failed: {e}", fg="red", err=True)
        sys.exit(1)


def main():
    """Main entry point."""
    cli(obj={})


if __name__ == "__main__":
    main()