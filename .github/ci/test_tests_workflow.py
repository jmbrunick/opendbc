import re
import unittest
from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[1] / "workflows" / "tests.yml"
NAP_JOB_CONDITION = "".join((
  "if: ${{ startsWith(github.ref_name, 'nap-') || startsWith(github.base_ref, 'nap-') || ",
  "startsWith(github.ref_name, 'naponsp-') || startsWith(github.base_ref, 'naponsp-') }}",
))
UPSTREAM_JOB_CONDITION = "".join((
  "if: ${{ !startsWith(github.ref_name, 'nap-') && !startsWith(github.base_ref, 'nap-') && ",
  "!startsWith(github.ref_name, 'naponsp-') && !startsWith(github.base_ref, 'naponsp-') }}",
))


def indented_block(document: str, header: str) -> str:
  lines = document.splitlines()
  header_index = lines.index(header)
  header_indent = len(header) - len(header.lstrip())
  block = []
  for line in lines[header_index + 1:]:
    line_indent = len(line) - len(line.lstrip())
    if line.strip() and line_indent <= header_indent:
      break
    block.append(line)
  return "\n".join(block)


def normalized_lines(block: str) -> set[str]:
  return {line.strip().removesuffix("\\").rstrip() for line in block.splitlines()}


class TestWorkflowContract(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.workflow = WORKFLOW_PATH.read_text()

  def test_nap_branches_run_on_push(self):
    push_config = indented_block(self.workflow, "  push:")
    push_branches = indented_block(push_config, "    branches:")

    self.assertRegex(push_branches, re.compile(r"^\s*-\s+['\"]?nap-\*['\"]?\s*$", re.MULTILINE))
    self.assertRegex(push_branches, re.compile(r"^\s*-\s+['\"]?naponsp-\*['\"]?\s*$", re.MULTILINE))

  def test_nap_gate_routes_pushes_and_pull_requests(self):
    nap_job = indented_block(self.workflow, "  nap_tests:")
    nap_lines = normalized_lines(nap_job)

    self.assertIn("name: NAP build and safety", nap_lines)
    self.assertIn(NAP_JOB_CONDITION, nap_lines)

  def test_upstream_jobs_are_isolated_from_nap_branches(self):
    for job_header in ("  tests:", "  safety_tests:", "  mutation:", "  test_models:"):
      with self.subTest(job=job_header):
        job = indented_block(self.workflow, job_header)
        self.assertEqual(re.findall(r"^    if: .+$", job, re.MULTILINE), [f"    {UPSTREAM_JOB_CONDITION}"])

  def test_nap_gate_pins_build_lint_and_safety_suites(self):
    nap_job = indented_block(self.workflow, "  nap_tests:")
    required_steps = (
      ("    - name: Build NAP opendbc", ("scons -j$(nproc)",)),
      ("    - name: Lint NAP implementation and gates", (
        "ruff check",
        "opendbc/safety/tests/test_tesla_preap_radar_carconfig.py",
      )),
      ("    - name: Run NAP car and safety suites", (
        "opendbc/car/tesla/preap/tests/",
        "opendbc/safety/tests/test_tesla_preap.py",
        "opendbc/safety/tests/test_tesla_preap_blinker.py",
        "opendbc/safety/tests/test_tesla_preap_lat_yield.py",
        "opendbc/safety/tests/test_tesla_preap_radar_carconfig.py",
        "opendbc/safety/tests/test_mg.py",
      )),
      ("    - name: Run lateral-yield mutation checks", (
        "python .github/ci/tesla_preap_lat_yield_mutations.py",
      )),
    )

    for step_header, required_commands in required_steps:
      with self.subTest(step=step_header):
        step = indented_block(nap_job, step_header)
        self.assertNotRegex(step, r"^\s+(?:if|continue-on-error):", msg=f"{step_header} must be unconditional")
        for command in required_commands:
          self.assertIn(command, step)

  def test_nap_gate_cannot_be_skipped_or_soft_failed(self):
    nap_job = indented_block(self.workflow, "  nap_tests:")

    self.assertEqual(re.findall(r"^    if: .+$", nap_job, re.MULTILINE), [f"    {NAP_JOB_CONDITION}"])
    self.assertIsNone(re.search(r"^\s+continue-on-error:", nap_job, re.MULTILINE))

  def test_focused_longitudinal_job_is_present(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIn("name: Tesla Pre-AP longitudinal regressions", normalized_lines(focused_job))

  def test_historical_mutations_run_in_focused_job(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIn(
      "python .github/ci/tesla_preap_longitudinal_mutations.py",
      normalized_lines(focused_job),
    )

  def test_focused_longitudinal_tests_are_pinned(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    required_test_paths = (
      "opendbc/car/tesla/preap/tests/test_pedal_authority.py",
      "opendbc/car/tesla/preap/tests/test_longitudinal_tuning.py",
      "opendbc/car/tesla/preap/tests/test_virtual_das.py",
      "opendbc/car/tesla/preap/tests/test_vdas_grade_control.py",
      "opendbc/car/tesla/preap/tests/test_accel_limits.py",
      "opendbc/car/tesla/preap/tests/test_engage_grace.py",
    )
    for test_path in required_test_paths:
      self.assertIn(test_path, normalized_lines(focused_job))

  def test_focused_longitudinal_job_cannot_be_skipped_or_soft_failed(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIsNone(re.search(r"^\s*(?:if|continue-on-error)\s*:", focused_job, re.MULTILINE))

  def test_model_job_uses_supported_openpilot_setup(self):
    model_job = indented_block(self.workflow, "  test_models:")
    model_lines = normalized_lines(model_job)

    self.assertIn("- run: ./tools/op.sh setup", model_lines)
    self.assertNotIn("uses: ./.github/workflows/setup-with-retry", model_lines)
    self.assertNotIn("setup-step.outputs.duration", model_job)
    self.assertLess(model_job.index("repository: 'commaai/openpilot'"), model_job.index("- run: ./tools/op.sh setup"))
    self.assertLess(model_job.index("- run: ./tools/op.sh setup"), model_job.index("- run: rm -rf opendbc_repo/"))

  def test_model_job_uses_current_openpilot_layout(self):
    model_job = indented_block(self.workflow, "  test_models:")
    model_lines = normalized_lines(model_job)
    model_test_command = " ".join((
      "run: MAX_EXAMPLES=1 pytest --continue-on-collection-errors --durations=0 --durations-min=5 -n logical",
      "openpilot/selfdrive/car/tests/test_models.py",
    ))

    self.assertIn("CI: 1", model_lines)
    self.assertIn(
      "run: scons -j$(nproc) openpilot/common/ openpilot/cereal/ openpilot/selfdrive/pandad/ msgq_repo/ opendbc_repo",
      model_lines,
    )
    self.assertIn(model_test_command, model_lines)
    for obsolete_variable in ("BASE_IMAGE:", "BUILD:", "RUN:", "PYTEST:"):
      self.assertNotIn(obsolete_variable, model_job)


if __name__ == "__main__":
  unittest.main()
