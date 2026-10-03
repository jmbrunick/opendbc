"""Lateral-yield (A cancels / B never cancels) mutations for the Tesla Pre-AP gate.

Panda infers "lateral yielded" from the 0x488 type-1 send history (see
opendbc/safety/modes/tesla_preap_latyield.h and opendbc/car/tesla/preap/lat_yield.py).
Each mutation breaks one piece of that rule; it is KILLED only by assertion
failures. Sources are restored byte-identically and libsafety is rebuilt after
every C mutation; any survivor or invalid run fails the job.

The CarController side (stamp / block) is mutation-tested in openpilot, where
CarController.apply can run against real cereal structs.
"""
import importlib.util
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


_PARENT_PATH = Path(__file__).resolve().parent / "tesla_preap_longitudinal_mutations.py"
_spec = importlib.util.spec_from_file_location("tesla_preap_longitudinal_mutations", _PARENT_PATH)
assert _spec is not None and _spec.loader is not None
parent = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = parent
_spec.loader.exec_module(parent)

REPO_ROOT = parent.REPO_ROOT
SAFETY_T = "opendbc/safety/tests/test_tesla_preap_lat_yield.py"
CARD_T = "opendbc/car/tesla/preap/tests/test_lat_yield.py"
HEADER = "opendbc/safety/modes/tesla_preap_latyield.h"
RX = "opendbc/safety/modes/tesla_preap_rx.h"
TX = "opendbc/safety/modes/tesla_preap_tx.h"
LY = "opendbc/car/tesla/preap/lat_yield.py"
ENG = "opendbc/car/tesla/preap/engagement.py"
CS = "opendbc/car/tesla/preap/carstate.py"
EXPECTED_COUNT = 16


@dataclass(frozen=True)
class Mutation:
  name: str
  source_path: str
  edits: tuple[tuple[bytes, bytes], ...]
  test_nodes: tuple[str, ...]

  @property
  def is_c(self) -> bool:
    return self.source_path.endswith(".h")


_WINDOWS = (
  b"#define PREAP_LAT_RECENT_US       100000U\n" +
  b"// A lapse this long in the active stream is a yield; the stream restarting\n" +
  b"// afterwards starts the re-arm grace.\n" +
  b"#define PREAP_LAT_GAP_US          100000U\n"
)


def _windows(us: int) -> tuple[tuple[bytes, bytes], ...]:
  return ((_WINDOWS, _WINDOWS.replace(b"100000U", f"{us}U".encode())),)


MUTATIONS = (
  Mutation("panda-always-cancel", RX,
           ((b"if (!controls_allowed || preap_lat_in_full_control()) {", b"if (true) {"),),
           (SAFETY_T,)),
  Mutation("panda-never-cancel", HEADER,
           ((b"  return (microsecond_timer_get() - preap_lat_last_us) <= PREAP_LAT_RECENT_US;",
             b"  return false;"),),
           (SAFETY_T,)),
  Mutation("panda-window-0ms", HEADER, _windows(1), (SAFETY_T,)),
  Mutation("panda-window-1000ms", HEADER, _windows(1000000), (SAFETY_T,)),
  Mutation("panda-no-regrab-grace", HEADER,
           ((b"#define PREAP_LAT_GRACE_US       1200000U", b"#define PREAP_LAT_GRACE_US       1U"),),
           (SAFETY_T,)),
  Mutation("panda-grace-on-first-engage", HEADER,
           ((b"    preap_lat_seen = true;\n    preap_lat_gap = false;\n    preap_lat_grace = false;\n",
             b"    preap_lat_seen = true;\n    preap_lat_gap = false;\n    preap_lat_grace = true;\n" +
             b"    preap_lat_grace_start_us = now;\n"),),
           (SAFETY_T,)),
  Mutation("panda-grace-never-expires", HEADER,
           ((b"  if (preap_lat_grace && ((now - preap_lat_grace_start_us) >= PREAP_LAT_GRACE_US)) {",
             b"  if (false) {"),),
           (SAFETY_T,)),
  Mutation("panda-history-survives-disengage", RX,
           ((b"  if (!controls_allowed) {\n    preap_lat_yield_reset();", b"  if (false) {\n    preap_lat_yield_reset();"),),
           (SAFETY_T,)),
  Mutation("panda-b-edge-does-not-block-steering", TX,
           ((b"if (steer_control_enabled && preap_lat_block) {", b"if (false) {"),),
           (SAFETY_T,)),
  Mutation("panda-stamp-not-recorded", TX,
           ((b"  } else if (active_steer_tx) {", b"  } else if (active_steer_tx && false) {"),),
           (SAFETY_T,)),
  Mutation("panda-type0-counts-as-lateral", TX,
           ((b"    active_steer_tx = steer_control_enabled;", b"    active_steer_tx = true;"),),
           (SAFETY_T,)),
  Mutation("card-window-shorter-than-panda", LY,
           ((b"RECENT_S = 0.15\n", b"RECENT_S = 0.05\n"),),
           (CARD_T,)),
  Mutation("card-no-grace", LY,
           ((b"GRACE_S = 1.0\n", b"GRACE_S = 0.0\n"),),
           (CARD_T,)),
  Mutation("card-b-branch-removed", ENG,
           ((b"        and not lat_full_control and self.cruiseEnabled):",
             b"        and False and self.cruiseEnabled):"),),
           (CARD_T,)),
  Mutation("card-ignores-lat-state", CS,
           ((b"handle_steering_disengage(ret.steeringDisengage, cs.lat_full_control)",
             b"handle_steering_disengage(ret.steeringDisengage, True)"),),
           (CARD_T,)),
  Mutation("panda-generic-rx-check-still-drops", RX,
           ((b"(preap_blinker_turn_blocks_disengage() || !preap_lat_in_full_control());",
             b"preap_blinker_turn_blocks_disengage();"),),
           (SAFETY_T,)),
)


def build_libsafety() -> None:
  result = subprocess.run(["scons", "-j4", "opendbc/safety/tests/libsafety"], cwd=REPO_ROOT,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
  if result.returncode != 0:
    print(result.stdout)
    raise RuntimeError("libsafety build failed")


def apply(mutation: Mutation) -> tuple[Path, bytes]:
  path = REPO_ROOT / mutation.source_path
  original = path.read_bytes()
  mutated = original
  for old, new in mutation.edits:
    if mutated.count(old) != 1:
      raise RuntimeError(f"{mutation.name}: expected one match in {mutation.source_path}, found {mutated.count(old)}")
    mutated = mutated.replace(old, new, 1)
  if mutated == original:
    raise RuntimeError(f"{mutation.name}: mutation did not change the source")
  path.write_bytes(mutated)
  return path, original


def main() -> int:
  if len(MUTATIONS) != EXPECTED_COUNT:
    print(f"INVALID: expected {EXPECTED_COUNT} lateral-yield mutations, found {len(MUTATIONS)}")
    return 1
  baseline_nodes = (SAFETY_T, CARD_T)
  with tempfile.TemporaryDirectory(prefix="tesla-preap-lat-yield-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    build_libsafety()
    baseline_xml = temp_root / "baseline.xml"
    baseline = parent.run_pytest(REPO_ROOT, baseline_nodes, baseline_xml)
    if baseline.returncode != 0:
      print("BASELINE FAILED: lateral-yield tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      print(f"BASELINE PASS: {len(parent.junit_testcases(baseline_xml))} lateral-yield tests")
    except parent.JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1

    survivors = []
    for mutation in MUTATIONS:
      path = original = None
      result = error = None
      restored = False
      xml = temp_root / f"{mutation.name}.xml"
      try:
        path, original = apply(mutation)
        if mutation.is_c:
          build_libsafety()
        result = parent.run_pytest(REPO_ROOT, mutation.test_nodes, xml)
      except Exception as exc:  # pragma: no cover - failure reporting path
        error = exc
      finally:
        if path is not None and original is not None:
          path.write_bytes(original)
          restored = path.read_bytes() == original
          if mutation.is_c:
            build_libsafety()
      if not restored:
        print(f"INVALID: {mutation.name} source restoration was not byte-identical ({error})")
        return 1
      if error is not None or result is None:
        print(f"INVALID: {mutation.name} could not run: {error}")
        return 1
      try:
        testcases = parent.junit_testcases(xml)
      except parent.JUnitReportError as exc:
        print(f"INVALID: {mutation.name} {exc}")
        return 1
      if result.returncode == 1 and parent.has_only_assertion_failures(testcases):
        print(f"KILLED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      elif result.returncode == 0:
        survivors.append(mutation.name)
        print(f"SURVIVED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      else:
        print(f"INVALID: {mutation.name} exited without assertion-only failures (pytest status {result.returncode})")
        print(result.stdout)
        return 1

    if survivors:
      print(f"Lateral-yield mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} lateral-yield mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
