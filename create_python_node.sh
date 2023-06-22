#!/bin/bash

touch ./scripts/$1.py
echo "install(PROGRAMS
	scripts/$1.py
	DESTINATION lib/\${PROJECT_NAME}
)" >> CMakeLists.txt

cat ./scripts/node_py.py > ./scripts/$1.py
