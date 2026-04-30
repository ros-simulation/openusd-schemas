"""CLI entry point: ``python -m compliance_checker`` or ``usd-check``."""

from __future__ import annotations

import logging
import sys

import click
from rich.console import Console

from .checker import ComplianceChecker
from .rep_info import num, title
from .report import Report, Severity

console = Console()
err_console = Console(stderr=True)

title = f"REP-{num} {title}"

_SEVERITY_STYLE = {
    Severity.ERROR: "bold red",
    Severity.WARNING: "yellow",
    Severity.INFO: "dim",
}

_SEVERITY_ICON = {
    Severity.ERROR: "[bold red]ERROR  [/]",
    Severity.WARNING: "[yellow]WARNING[/]",
    Severity.INFO: "[dim]INFO   [/]",
}


def _severity_from_str(value: str) -> Severity:
    try:
        return Severity(value.lower())
    except ValueError:
        raise click.BadParameter(
            f"Unknown severity '{value}'. Choose: error, warning, info."
        )


@click.command(name="usd-check")
@click.argument("asset", type=click.Path(exists=True, readable=True))
@click.option(
    "--extensions",
    is_flag=True,
    default=False,
    help=(
        "Include §4 extension schema checks: ExtendedPhysics* and RosControl* schemas. "
        "By default only core §1/§2 checks run."
    ),
)
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
    "--no-extensions",
    "no_extensions",
    is_flag=True,
    default=False,
    help="Disable loading of third-party extension check plug-ins.",
)
@click.option(
    "--verbose", "-v", is_flag=True, default=False, help="Enable debug logging."
)
def main(
    asset: str,
    extensions: bool,
    export: bool,
    sections: str | None,
    severity: str,
    fail_on: str,
    output_format: str,
    no_extensions: bool,
    verbose: bool,
) -> None:
    """Validate a USD asset against the REP-XXXX interoperability standard.

    ASSET is the path to a .usd / .usda / .usdc file.

    By default only the core §1 (units, structure, physics) and §2 (base ROS
    schemas) checks run.  Use --extensions to also validate §4 extension schemas
    (ExtendedPhysics*, RosControl*) and --export to add §3 export/conversion
    checks.
    """
    if verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    min_severity = _severity_from_str(severity)
    fail_severity = _severity_from_str(fail_on)

    section_list = [s.strip() for s in sections.split(",")] if sections else None

    try:
        checker = ComplianceChecker.from_path(
            asset,
            include_export=export,
            include_extended=extensions,
            sections=section_list,
            include_extensions=not no_extensions,
        )
    except FileNotFoundError as exc:
        err_console.print(f"[bold red]Error:[/] {exc}")
        sys.exit(2)

    report = checker.run()

    # ------------------------------------------------------------------ #
    # Output                                                                #
    # ------------------------------------------------------------------ #

    if output_format == "json":
        # For JSON, ignore min_severity filter (output everything)
        click.echo(report.to_json())
    else:
        _print_text_report(report, min_severity)

    # ------------------------------------------------------------------ #
    # Exit code                                                             #
    # ------------------------------------------------------------------ #

    _severity_order = {Severity.ERROR: 0, Severity.WARNING: 1, Severity.INFO: 2}
    worst = min(
        (v.severity for v in report.violations),
        key=lambda s: _severity_order[s],
        default=Severity.INFO,
    )
    if _severity_order[worst] <= _severity_order[fail_severity]:
        sys.exit(1)


def _print_text_report(report: Report, min_severity: Severity) -> None:
    _sev_order = {Severity.ERROR: 0, Severity.WARNING: 1, Severity.INFO: 2}
    min_order = _sev_order[min_severity]

    visible = [v for v in report.violations if _sev_order[v.severity] <= min_order]

    # Header
    console.rule(f"[bold]{title} Compliance Report[/]")
    console.print(f"[dim]Asset:[/] {report.asset_path}")
    console.print(
        f"[bold red]{len(report.errors)} error(s)[/]  "
        f"[yellow]{len(report.warnings)} warning(s)[/]  "
        f"[dim]{len(report.infos)} info[/]  "
        f"({'[green]PASSED[/]' if not report.has_errors() else '[red]FAILED[/]'})"
    )

    if not visible:
        console.print("\n[green]No violations at the selected severity level.[/]")
        return

    # Group by section
    by_section = {}
    for v in visible:
        by_section.setdefault(v.section, []).append(v)

    for section_key in sorted(by_section):
        console.print(f"\n[bold]§{section_key}[/]")
        for v in by_section[section_key]:
            style = _SEVERITY_STYLE[v.severity]
            icon = _SEVERITY_ICON[v.severity]
            console.print(f"  {icon} [{style}][{v.check_id}][/]  [dim]{v.prim_path}[/]")
            # Print dynamic text with markup disabled to avoid Rich parsing
            # characters like '[' and ']' in violation content.
            console.print(f"         {v.message}", markup=False)
            if v.suggestion:
                console.print(
                    f"         Suggestion: {v.suggestion}",
                    style="italic dim",
                    markup=False,
                )

    # Footer summary
    console.rule()
    console.print(
        f"[dim]Checks run against:[/] {report.asset_path}\n"
        f"[dim]Violations shown (>= {min_severity.value}):[/] {len(visible)} of {len(report.violations)}"
    )


if __name__ == "__main__":
    main()
