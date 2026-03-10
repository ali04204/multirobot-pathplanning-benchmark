from simple_parsing import ArgumentParser

from multi_robot_multi_goal_planning.problems import get_env_by_name
from multi_robot_multi_goal_planning.problems.registry import get_all_environments


def _env_names():
    envs = get_all_environments()

    if isinstance(envs, dict):
        names = list(envs.keys())
        names.sort()
        return names

    names = []
    for item in envs:
        if isinstance(item, str):
            names.append(item)
        elif hasattr(item, "name"):
            names.append(getattr(item, "name"))
        else:
            names.append(str(item))

    names = sorted(set(names))
    return names


def main():
    parser = ArgumentParser()
    parser.add_argument("--mode", choices=["list_all", "show"], default="list_all")
    parser.add_argument("--env", default=None)
    args = parser.parse_args()

    if args.mode == "list_all":
        for n in _env_names():
            print(n)
        return

    if args.mode == "show":
        if args.env is None:
            raise ValueError("Pass --env ENV_NAME")

        env = get_env_by_name(args.env)
        env.show(blocking=True)
        if hasattr(env, "close"):
            env.close()
        return


if __name__ == "__main__":
    main()