import os
import sys
from pathlib import Path
import json
import argparse
import platform

from conda.cli.python_api import run_command

commands_app = {"mercurial": ["hg"], "tortoisehg": ["hg", "thg"]}
apps_with_app = ["mercurial"]

if os.name == "nt":
    data_dir = "AppData"
else:
    data_dir = ".local/share"
    if platform.system() == "Darwin":
        bash_config = Path.home() / ".bash_profile"
    else:
        bash_config = Path.home() / ".bashrc"
    if not bash_config.exists():
        bash_config.touch()

data_dir = Path.home() / data_dir
data_dir.mkdir(exist_ok=True, parents=True)
path_data = data_dir / "conda-app.json"


def query_yes_no(question, default="yes"):
    """Ask a yes/no question and return the answer.

    Parameters
    ----------

    question : string
       String that is presented to the user.

    default : bool
       The default answer if the user just hits <Enter>.
       It must be "yes" (the default), "no" or None (meaning
       an answer is required of the user).

    Returns
    -------

    answer : bool
       The returned answer.
    """
    valid = {"yes": True, "y": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        print(question + prompt, flush=True, end="")
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]

        elif choice in valid:
            return valid[choice]

        else:
            print(
                "Please respond with 'yes' or 'no' " "(or 'y' or 'n').",
                flush=True,
            )


def modif_config_file(path_config, line_config):
    path_config = Path(path_config)
    if not line_config.endswith("\n"):
        line_config = line_config + "\n"
    if path_config.exists():
        with open(path_config) as file:
            lines = file.readlines()
        if lines and lines[-1] and not lines[-1].endswith("\n"):
            lines[-1] = lines[-1] + "\n"
        if line_config not in lines:
            print(
                f"Add line \n{line_config.strip()}\n"
                f"at the end of file {path_config}"
            )

            with open(
                path_config.with_name(path_config.name + ".orig"), "w"
            ) as file:
                file.write("".join(lines))

            with open(path_config, "a") as file:
                file.write("\n# line added by conda-app\n" + line_config)


def get_conda_data():
    result = run_command("info", "--json")
    return json.loads(result[0])


def get_env_names(conda_data):
    envs = conda_data["envs"]
    path_root = conda_data["root_prefix"]
    env_names = []
    for path_env in envs:
        if path_env.startswith(path_root):
            path_env = path_env[len(path_root) + 1 :]
        if path_env.startswith("envs" + os.path.sep):
            path_env = path_env[5:]

        env_names.append(path_env)
    return env_names


def install_app(app_name):

    package_name = app_name + "-app"

    try:
        result = run_command("search", package_name, "--json")
    except Exception:
        if app_name in apps_with_app:
            print(
                "Please first add the conda-forge channel with:\n"
                "conda config --add channels conda-forge"
            )
            sys.exit(1)

        package_name = app_name
        result = run_command("search", package_name, "--json")
    # else:
    #     data = json.loads(result[0])
    #     print(data[package_name][-1])

    conda_data = get_conda_data()
    # print(conda_data)
    path_root = conda_data["root_prefix"]

    if conda_data["root_writable"]:
        if os.name == "nt":
            # quickfix: I wasn't able to permanently set the PATH on Windows
            path_bin = Path(path_root) / "condabin"
        else:
            path_bin = Path(path_root) / "condabin/app"
    else:
        if not os.name == "nt":
            path_bin = Path.home() / ".local/bin/conda-app"
        else:
            raise NotImplementedError
    path_bin.mkdir(exist_ok=True, parents=True)

    export_path_posix = f"export PATH={path_bin}:$PATH\n"
    # bash
    modif_config_file(bash_config, export_path_posix)

    # zsh
    modif_config_file(Path.home() / ".zshrc", export_path_posix)

    # fish
    modif_config_file(
        Path.home() / ".config/fish/config.fish",
        f"set -gx PATH {path_bin} $PATH\n",
    )

    env_names = get_env_names(conda_data)
    env_name = "_env_" + app_name
    env_path = Path(path_root) / "envs" / env_name

    if env_name not in env_names:
        print(
            (
                "Create conda environment "
                f"{env_name} with package {package_name}"
                " (it can be long)..."
            ),
            flush=True,
        )

        result = run_command("create", "-n", env_name, package_name, "--json")
        try:
            data_create = json.loads(result[0])
        except json.decoder.JSONDecodeError:
            print("warning: json.decoder.JSONDecodeError")
            # print(result[0])
        else:
            env_path = Path(data_create["prefix"])

        print(f"Conda environment {env_name} created!")

        if app_name == "mercurial":
            print("Install hg-git with pip... ", end="", flush=True)
            run_command(
                "run",
                "-n",
                env_name,
                "pip",
                "install",
                "hg+https://bitbucket.org/durin42/hg-git",
                "--no-cache-dir",
            )
            print("done")

        try:
            commands = commands_app[app_name]
        except KeyError:
            commands = [app_name]

        for command in commands:
            if os.name == "nt":
                with open(path_bin / (command + ".bat"), "w") as file:
                    file.write(
                        "@echo off\n"
                        f"call conda activate {env_name}\n"
                        f"{command} %*\n"
                        "call conda deactivate\n"
                    )
            else:
                path_command = env_path / "bin" / command
                path_symlink = path_bin / command
                if path_symlink.exists():
                    path_symlink.unlink()
                path_symlink.symlink_to(path_command)

        if os.name == "nt":
            txt = "T"
        else:
            txt = "Open a new terminal and t"

        print(
            f"{app_name} should now be installed in\n{env_path}\n"
            + txt
            + f"he command(s) {commands} should be available."
        )

        add_to_app_list(app_name)
    else:
        print(
            f"environment {env_name} already exists in \n{env_path}\n"
            f"To reinstall or update {app_name}, first uninstall it with:\n"
            f"conda-app uninstall {app_name}"
        )


def load_data():
    if path_data.exists():
        with open(path_data) as file:
            data = json.load(file)
    else:
        data = {"installed_apps": []}

    return data


def add_to_app_list(app_name):
    data = load_data()
    if app_name not in data["installed_apps"]:
        data["installed_apps"].append(app_name)
    with open(path_data, "w") as file:
        json.dump(data, file)


def list_apps():
    data = load_data()
    print("Installed applications:\n", data["installed_apps"])


def uninstall_app(app_name):
    conda_data = get_conda_data()
    env_names = get_env_names(conda_data)

    env_name = "_env_" + app_name

    if env_name not in env_names:
        print("Nothing to do")
        return

    if query_yes_no(f"The application {app_name} will be uninstalled.\nProceed"):
        import shutil

        path_root = conda_data["root_prefix"]
        env_path = Path(path_root) / "envs" / env_name
        shutil.rmtree(env_path, ignore_errors=True)
        print(f"Directory {env_path} removed")


commands = ["install", "list", "uninstall"]


class SmartFormatter(argparse.HelpFormatter):
    def _split_lines(self, text, width):
        if text.startswith("R|"):
            return text[2:].splitlines()
        # this is the RawTextHelpFormatter._split_lines
        return argparse.HelpFormatter._split_lines(self, text, width)


def main():

    parser = argparse.ArgumentParser(
        prog="conda-app",
        description="Install applications using conda.",
        formatter_class=SmartFormatter,
    )
    parser.add_argument(
        "command",
        type=str,
        help=(
            "R|Can be in:\n- install: install an application\n"
            "- list: list applications installed with conda-app\n"
        ),
    )

    parser.add_argument(
        "package_spec",
        type=str,
        nargs="?",
        default=None,
        help="Package to install.",
    )

    args = parser.parse_args()

    if args.command not in commands:
        print(f"command {args.command} unknown")
        sys.exit(1)
    elif args.command == "install":
        install_app(args.package_spec)
    elif args.command == "list":
        list_apps()
    elif args.command == "uninstall":
        uninstall_app(args.package_spec)
    else:
        raise NotImplementedError


if __name__ == "__main__":
    main()
