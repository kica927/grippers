from setuptools import find_packages, setup

package_name = "grippers_vla"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="11306260+liangfuyuan@user.noreply.gitee.com",
    description="SmolVLA 정책을 텔레옵 팔로워에 물리는 노드 (스트레치)",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "smolvla_policy_node = grippers_vla.smolvla_policy_node:main",
        ],
    },
)
