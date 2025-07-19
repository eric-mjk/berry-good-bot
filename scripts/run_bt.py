#scripts/run_bt.py

import subprocess, hydra, os, sys

@hydra.main(version_base=None, config_path="../config", config_name="default")
def main(cfg):
    # cfg에서 직접 XML 파일의 절대 경로를 가져옵니다.
    xml_absolute_path = cfg.bt.tree_file_path # YAML에서 정의한 tree_file_path 사용

    node_types_to_register = ",".join(cfg.bt.get("register_node_types", []))

    env_vars = os.environ.copy()

    args = [
        "ros2", "run", "strawberry_task_planner", "run_bt_node",
        "--ros-args", "-p", f"bt_tree_file:={xml_absolute_path}" # 절대 경로 그 자체를 파라미터로 전달
    ]

    if node_types_to_register:
        args.extend(["-p", f"register_node_types:=[{node_types_to_register}]"])

    print(f"Executing command: {' '.join(args)}") # 디버깅을 위해 추가
    subprocess.run(args, env=env_vars)


if __name__ == "__main__":
    main()