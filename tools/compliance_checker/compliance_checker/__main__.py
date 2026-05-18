"""CLI entry point: ``python -m compliance_checker`` or ``usd-check``."""

from __future__ import annotations

import logging
import sys

import click
from pxr import Sdf, Usd, UsdValidation
from rich.console import Console

from .checks import base as check_base
from .rep_info import num, title
from .report import _prim_path_from_error, _section_from_error, _severity_label, errors_to_json

console = Console()
err_console = Console(stderr=True)

_title = f"REP-{num} {title}"

_SEVERITY_STYLE = {
    "error": "bold red",
    "warning": "yellow",
    "info": "dim",
}

_SEVERITY_ICON = {
    "error": "[bold red]ERROR  [/]",
    "warning": "[yellow]WARNING[/]",
    "info": "[dim]INFO   [/]",
}

_SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}


@click.command(name="usd-check")
@click.argument("asset", type=click.Path(exists=True, readable=True))
@click.option(
    "--export",
    is_flag=True,
    default=False,
    help="Include §3 export/conversion checks (mesh/texture traversal; slower).",
)
@click.option(
    "--sections",
    default=None,
    metavar="LIST",
    help=(
        "Comma-separated section prefixes to run, e.g. '1.1,2'. "
        "Omit to run all sections."
    ),
)
@click.option(
    "--severity",
    default="warning",
    metavar="LEVEL",
    help="Minimum severity to display: error | warning | info. (default: warning)",
)
@click.option(
    "--fail-on",
    "fail_on",
    default="error",
    metavar="LEVEL",
    help="Exit non-zero when any violation reaches this severity. (default: error)",
)
@click.option(
    "--format",
    "output_format",
    default="text",
    type=click.Choice(["text", "json"], case_sensitive=False),
    help="Output format. (default: text)",
)
@click.option(
    "--verbose", "-v", is_flag=True, default=False, help="Enable debug logging."
)
def main(
    asset: str,
    export: bool,
    sections: str | None,
    severity: str,
    fail_on: str,
    output_format: str,
    verbose: bool,
) -> None:
    """Validate a USD asset against the REP-0158 interoperability standard.

    ASSET is the path to a .usd / .usda / .usdc file.

    By default only the core §1 (units, structure, physics) and §2 (base ROS
    schemas) checks run.  Use --export to also add §3 export/conversion checks.
    """
    if verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    min_severity = severity.lower()
    fail_severity = fail_on.lower()
    if min_severity not in _SEVERITY_ORDER:
        raise click.BadParameter(f"Unknown severity '{severity}'.")
    if fail_severity not in _SEVERITY_ORDER:
        raise click.BadParameter(f"Unknown severity '{fail_on}'.")

    section_list = [s.strip() for s in sections.split(",")] if sections else None

    try:
        stage = Usd.Stage.Open(asset, Usd.Stage.LoadAll)
        if not stage:
            raise FileNotFoundError(f"Could not open USD stage: {asset!r}")
    except Exception as exc:
        err_console.print(f"[bold red]Error:[/] {exc}")
        sys.exit(2)

    keywords = _build_keywords(
        include_export=export,
        sections=section_list,
    )

    validators = check_base.get_validators_for_keywords(keywords)
    ctx = UsdValidation.ValidationContext(validators)
    all_errors = ctx.Validate(stage)

    if section_list:
        all_errors = [
            e for e in all_errors
            if any(_section_from_error(e).startswith(s) for s in section_list)
        ]

    if output_format == "json":
        click.echo(errors_to_json(asset, all_errors))
    else:
        _print_text_report(asset, all_errors, min_severity)

    worst = min(
        (_severity_label(e) for e in all_errors),
        key=lambda s: _SEVERITY_ORDER.get(s, 2),
        default="info",
    )
    if _SEVERITY_ORDER.get(worst, 2) <= _SEVERITY_ORDER[fail_severity]:
        sys.exit(1)


def _build_keywords(
    include_export: bool,
    sections: list[str] | None,
) -> list[str]:
    if sections:
        return [f"rep0158:{s}" for s in sections]

    core = ["rep0158:1.1", "rep0158:1.2", "rep0158:1.3",
            "rep0158:2.1", "rep0158:2.2", "rep0158:2.4",
            "rep0158:2.5", "rep0158:2.6", "rep0158:2.7",
            "rep0158:2.8", "rep0158:2.10"]
    if include_export:
        core += ["rep0158:3.1", "rep0158:3.2", "rep0158:3.3",
                 "rep0158:3.4", "rep0158:3.6"]
    return core


def _print_text_report(
    asset_path: str,
    errors: list,
    min_severity: str,
) -> None:
    min_order = _SEVERITY_ORDER[min_severity]

    visible = [e for e in errors if _SEVERITY_ORDER.get(_severity_label(e), 2) <= min_order]

    n_errors = sum(1 for e in errors if _severity_label(e) == "error")
    n_warnings = sum(1 for e in errors if _severity_label(e) == "warning")
    n_infos = sum(1 for e in errors if _severity_label(e) == "info")

    console.rule(f"[bold]{_title} Compliance Report[/]")
    console.print(f"[dim]Asset:[/] {asset_path}")
    console.print(
        f"[bold red]{n_errors} error(s)[/]  "
        f"[yellow]{n_warnings} warning(s)[/]  "
        f"[dim]{n_infos} info[/]  "
        f"({'[green]PASSED[/]' if n_errors == 0 else '[red]FAILED[/]'})"
    )

    if not visible:
        console.print("\n[green]No violations at the selected severity level.[/]")
        return

    by_section: dict[str, list] = {}
    for e in visible:
        sec = _section_from_error(e)
        by_section.setdefault(sec, []).append(e)

    for section_key in sorted(by_section):
        console.print(f"\n[bold]§{section_key}[/]")
        for e in by_section[section_key]:
            sev = _severity_label(e)
            style = _SEVERITY_STYLE[sev]
            icon = _SEVERITY_ICON[sev]
            prim_path = _prim_path_from_error(e)
            check_id = e.GetName()
            console.print(f"  {icon} [{style}][{check_id}][/]  [dim]{prim_path}[/]")
            console.print(f"         {e.GetMessage()}", markup=False)

    console.rule()
    console.print(
        f"[dim]Checks run against:[/] {asset_path}\n"
        f"[dim]Violations shown (>= {min_severity}):[/] {len(visible)} of {len(errors)}"
    )


if __name__ == "__main__":
    main()
