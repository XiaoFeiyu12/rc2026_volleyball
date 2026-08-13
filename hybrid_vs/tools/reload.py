#!/usr/bin/env python3
"""方便每次改代码调参数编译使用"""
import subprocess
import sys
import argparse


def run(cmd: str, description: str) -> None:
    print(f"\n>>> {description}")
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print(f"[FAIL] {description}", file=sys.stderr)
        sys.exit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description="编译并重启 volleyball 容器")
    parser.add_argument("--skip-build", action="store_true", help="跳过编译，仅重启")
    parser.add_argument("--skip-restart", action="store_true", help="跳过重启，仅编译")
    args = parser.parse_args()

    if not args.skip_build:
        run(
            'docker exec -it volleyball-robot bash -c "colcon build --parallel-workers 4"',
            "编译中...",
        )

    if not args.skip_restart:
        run(
            "docker compose restart volleyball",
            "重启 volleyball 容器中...",
        )

    print("\n[DONE]")


if __name__ == "__main__":
    main()
