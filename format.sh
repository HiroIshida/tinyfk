#!/bin/bash
find python -name "*py"|xargs python3 -m autoflake -i --remove-all-unused-imports --remove-unused-variables --ignore-init-module-imports
python3 -m isort python/
python3 -m black python/
python3 -m flake8 python/
find src/ include/ test/ bench/ -name "*.cpp"|xargs clang-format -i
find src/ include/ test/ bench/ -name "*.hpp"|xargs clang-format -i
