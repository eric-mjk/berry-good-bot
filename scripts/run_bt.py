import subprocess, hydra, os, sys

@hydra.main(version_base=None, config_path="../config", config_name="default")
def main(cfg):
    xml = cfg.bt.tree_file
    plugins = ",".join(cfg.bt.get("plugins", []))
    subprocess.run([
        "ros2", "run", "strawberry_task_planner", "run_bt_node",
        "--ros-args", "-p", f"bt_tree:={xml}",
        "-p", f"plugins:={plugins}"
    ])
if __name__ == "__main__":
    main()
