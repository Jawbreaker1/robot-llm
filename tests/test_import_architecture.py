import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = (
    Path(__file__).resolve().parents[1] / "src" / "robot_agent"
)


def _module_name(path):
    relative = path.relative_to(PACKAGE_ROOT)
    parts = list(relative.with_suffix("").parts)
    if parts[-1] == "__init__":
        parts.pop()
    return ".".join(("robot_agent", *parts))


def _import_target(current_module, path, node):
    if node.level == 0:
        return node.module

    current_parts = current_module.split(".")
    package_parts = (
        current_parts
        if path.name == "__init__.py"
        else current_parts[:-1]
    )
    keep = len(package_parts) - (node.level - 1)
    if keep < 0:
        return None
    parts = package_parts[:keep]
    if node.module:
        parts.extend(node.module.split("."))
    return ".".join(parts)


class ImportArchitectureTests(unittest.TestCase):
    def test_robot_agent_module_graph_has_no_cycles(self):
        paths = sorted(PACKAGE_ROOT.rglob("*.py"))
        modules = {_module_name(path): path for path in paths}
        graph = {module: set() for module in modules}

        for module, path in modules.items():
            tree = ast.parse(
                path.read_text(encoding="utf-8"),
                filename=str(path),
            )
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name in modules:
                            graph[module].add(alias.name)
                elif isinstance(node, ast.ImportFrom):
                    target = _import_target(module, path, node)
                    if target in modules:
                        graph[module].add(target)
                    if target == "robot_agent":
                        for alias in node.names:
                            candidate = "{}.{}".format(target, alias.name)
                            if candidate in modules:
                                graph[module].add(candidate)

        visiting = []
        visited = set()

        def visit(module):
            if module in visiting:
                start = visiting.index(module)
                return visiting[start:] + [module]
            if module in visited:
                return None
            visiting.append(module)
            for dependency in sorted(graph[module]):
                cycle = visit(dependency)
                if cycle:
                    return cycle
            visiting.pop()
            visited.add(module)
            return None

        for module in sorted(graph):
            cycle = visit(module)
            self.assertIsNone(
                cycle,
                "robot_agent import cycle: {}".format(
                    " -> ".join(cycle or ())
                ),
            )
