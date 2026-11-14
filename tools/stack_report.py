#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen
"""Static stack report for a task entry point, from GCC's call-graph output.

Build with the measurement option first, in a build directory of its own:

    idf.py -B build_su -DHABINARI_STACK_USAGE=ON build

which adds -fstack-usage -fcallgraph-info=su to every object. Then:

    tools/stack_report.py build_su knxServiceTask Tp1DataLinkLayer::rxTaskFunc

A root is matched like an edge pattern (below); the shortest match wins. For each root it prints the deepest chain of frames that
the call graph resolves, the largest frames reachable from the root, recursion
and dynamically sized frames, and the indirect calls (function pointers,
std::function / InplaceFunction, virtuals the compiler did not devirtualise)
where the static chain stops.

The chain total is a LOWER bound: whatever runs behind an unresolved indirect
call is not in it. Known indirect edges can be stitched in by hand with
--edge 'caller-pattern -> callee-pattern' (repeatable) to follow a path the
compiler cannot see, e.g. a receive callback into the layer above, or from a
file with --edges (tools/stack_report.edges holds the KNX receive path).
Patterns match the demangled name (substring, or a glob with '*'), optionally
followed by '@file:line' to pick one of several same-named lambdas.
"""

import argparse
import fnmatch
import os
import re
import sys
from collections import defaultdict

INDIRECT = "__indirect_call"
NODE_RE = re.compile(r'node:\s*\{\s*title:\s*"((?:[^"\\]|\\.)*)"\s*label:\s*"((?:[^"\\]|\\.)*)"')
EDGE_RE = re.compile(r'edge:\s*\{\s*sourcename:\s*"((?:[^"\\]|\\.)*)"\s*targetname:\s*"((?:[^"\\]|\\.)*)"'
                     r'(?:\s*label:\s*"((?:[^"\\]|\\.)*)")?')
SIZE_RE = re.compile(r"(\d+) bytes \(([a-z, ]+)\)")
LOC_RE = re.compile(r"\\n([^\\]*:\d+:\d+)(?:\\n|$)")


class Node:
    __slots__ = ("title", "name", "where", "size", "kind", "defined")

    def __init__(self, title):
        self.title = title
        self.name = title
        self.where = ""
        self.size = 0
        self.kind = ""
        self.defined = False


def load(build_dir):
    nodes = {}
    edges = defaultdict(set)      # caller title -> callee titles
    indirect = defaultdict(list)  # caller title -> call-site locations
    ci_files = 0
    for root, _, files in os.walk(build_dir):
        for f in files:
            if not f.endswith(".ci"):
                continue
            ci_files += 1
            with open(os.path.join(root, f), encoding="utf-8", errors="replace") as fh:
                text = fh.read()
            for title, label in NODE_RE.findall(text):
                m = SIZE_RE.search(label)
                node = nodes.get(title)
                if node is None:
                    node = nodes[title] = Node(title)
                if m is None and node.defined:
                    continue  # a declaration seen after the definition
                # "name\nfile:line:col\nN bytes (kind)", where the name itself
                # can contain "\n" (string literals in template arguments).
                loc = LOC_RE.search(label)
                node.name = label[: loc.start()] if loc else label.split("\\n")[0]
                if m is not None:
                    node.where = loc.group(1) if loc else ""
                    node.size = max(node.size, int(m.group(1)))
                    node.kind = m.group(2)
                    node.defined = True
            for src, dst, where in EDGE_RE.findall(text):
                if dst == INDIRECT:
                    indirect[src].append(where)
                else:
                    edges[src].add(dst)
    if ci_files == 0:
        sys.exit(f"no .ci files under {build_dir}; build with -DHABINARI_STACK_USAGE=ON")
    return nodes, edges, indirect


def matches(node, pattern):
    # "name@file:line" also requires the definition's location to contain
    # "file:line": lambdas and their invokers often share one name.
    pattern, _, where = pattern.partition("@")
    if where and where not in node.where:
        return False
    if "*" in pattern:
        return fnmatch.fnmatchcase(node.name, pattern)
    return pattern in node.name


def find(nodes, pattern):
    hits = [n for n in nodes.values() if n.defined and matches(n, pattern)]
    if not hits:
        return None
    return min(hits, key=lambda n: len(n.name))


def analyse(nodes, edges, root):
    best = {}        # title -> (total, next title)
    recursive = set()
    on_stack = set()

    sys.setrecursionlimit(20000)

    def visit(t):
        if t in best:
            return best[t][0]
        if t in on_stack:
            recursive.add(t)
            return 0
        on_stack.add(t)
        own = nodes[t].size if t in nodes else 0
        total, nxt = own, None
        for callee in edges.get(t, ()):
            sub = visit(callee)
            if own + sub > total:
                total, nxt = own + sub, callee
        on_stack.discard(t)
        best[t] = (total, nxt)
        return total

    visit(root)
    chain = []
    t = root
    while t is not None:
        chain.append(t)
        t = best[t][1]
    return best[root][0], chain, set(best), recursive


def short(name, width=100):
    # Template arguments make most names unreadable; keep the outer shape.
    # Scanned from the end, because GCC cuts the front off very long labels,
    # which leaves unmatched '>' at the start; those simply never reopen.
    out, depth = [], 0
    for i in range(len(name) - 1, -1, -1):
        ch = name[i]
        if ch == ">" and not name.endswith("-", 0, i):
            depth += 1
            if depth == 1:
                out.append(">")
        elif ch == "<" and depth > 0:
            depth -= 1
            if depth == 0:
                out.append("<lambda" if name.startswith("lambda", i + 1) else "<")
        elif depth == 0:
            out.append(ch)
    name = "".join(reversed(out)).strip()
    if name[:1] in (">", ")", ",", ":"):
        name = "~" + name.lstrip(">),: ")  # the front was cut off by GCC
    return name if len(name) <= width else "..." + name[-(width - 3):]


def report(nodes, edges, indirect, root_pattern, top):
    root = find(nodes, root_pattern)
    if root is None:
        print(f"\n=== {root_pattern}: not found\n")
        return
    total, chain, reach, recursive = analyse(nodes, edges, root.title)
    print(f"\n=== {root.name}")
    print(f"deepest resolved chain: {total} bytes over {len(chain)} frames (lower bound)")
    for t in chain:
        n = nodes.get(t)
        size = n.size if n else 0
        where = n.where.rsplit("/", 1)[-1] if n else ""
        print(f"  {size:6d}  {short(n.name if n else t)}  [{where}]")
    print(f"\nlargest frames reachable ({len(reach)} functions):")
    for n in sorted((nodes[t] for t in reach if t in nodes), key=lambda n: -n.size)[:top]:
        print(f"  {n.size:6d}  {short(n.name)}  [{n.where.rsplit('/', 1)[-1]}]")
    dyn = [nodes[t] for t in reach if t in nodes and "dynamic" in nodes[t].kind]
    if dyn:
        print("\ndynamically sized frames:")
        for n in sorted(dyn, key=lambda n: -n.size):
            print(f"  {n.size:6d}  {n.kind:17s} {short(n.name)}")
    if recursive:
        print("\nrecursion (cycle cut, not counted twice):")
        for t in sorted(recursive):
            print(f"  {short(nodes[t].name if t in nodes else t)}")
    callers = [t for t in reach if indirect.get(t)]
    sites = sum(len(indirect[t]) for t in callers)
    print(f"\nunresolved indirect calls: {sites} sites in {len(callers)} functions "
          f"(the chain above stops at each of them)")
    ranked = sorted(callers, key=lambda t: -best_total(nodes, edges, t))
    for t in ranked[:top]:
        depth = best_total(nodes, edges, t)
        print(f"  {depth:6d}  {short(nodes[t].name if t in nodes else t)}  x{len(indirect[t])}")


_depth_cache = {}


def best_total(nodes, edges, t):
    if t not in _depth_cache:
        _depth_cache[t] = analyse(nodes, edges, t)[0]
    return _depth_cache[t]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build_dir")
    ap.add_argument("roots", nargs="+", help="demangled-name substrings of the task entry points")
    ap.add_argument("--edge", action="append", default=[], metavar="'CALLER -> CALLEE'",
                    help="stitch in an indirect call the compiler cannot resolve")
    ap.add_argument("--edges", action="append", default=[], metavar="FILE",
                    help="a file of 'CALLER -> CALLEE' lines (# comments)")
    ap.add_argument("--top", type=int, default=12)
    args = ap.parse_args()

    nodes, edges, indirect = load(args.build_dir)
    specs = list(args.edge)
    for path in args.edges:
        with open(path, encoding="utf-8") as fh:
            specs += [ln.strip() for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
    for spec in specs:
        a, _, b = spec.partition(" -> ")
        ca, cb = find(nodes, a.strip()), find(nodes, b.strip())
        if ca is None or cb is None:
            # One edge file serves every image; the other medium's are absent.
            print(f"not in this build: {spec}")
            continue
        edges[ca.title].add(cb.title)
        print(f"stitched: {short(ca.name, 60)} -> {short(cb.name, 60)}")
    for r in args.roots:
        _depth_cache.clear()
        report(nodes, edges, indirect, r, args.top)


if __name__ == "__main__":
    main()
