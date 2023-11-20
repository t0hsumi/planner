import re

from lab.parser import Parser


def error(content, props):
    if props["planner_exit_code"] == 0:
        props["error"] = "plan-found"
    else:
        props["error"] = "unsolvable-or-error"


def coverage(content, props):
    props["coverage"] = int(props["planner_exit_code"] == 0)


def get_plan(content, props):
    # All patterns are parsed before functions are called.
    if props.get("evaluations") is not None:
        props["plan"] = re.findall(r"^(?:step)?\s*\d+: (.+)$", content, re.M)


def get_times(content, props):
    props["times"] = re.findall(r"(\d+) msec", content)


def trivially_unsolvable(content, props):
    props["trivially_unsolvable"] = int(
        "No solution found." in content
    )


class TestParser(Parser):
    def __init__(self):
        super().__init__()
        self.add_pattern(
            "node", r"node: (.+)\n", type=str, file="driver.log", required=True
        )
        self.add_pattern(
            "planner_exit_code",
            r"run-planner exit code: (.+)\n",
            type=int,
            file="driver.log",
        )
        self.add_pattern("evaluations", r"evaluating (\d+) states")
        self.add_function(error)
        self.add_function(coverage)
        self.add_function(get_plan)
        self.add_function(get_times)
        self.add_function(trivially_unsolvable)
