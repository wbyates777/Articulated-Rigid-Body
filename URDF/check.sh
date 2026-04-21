if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <dir>"
    exit 1
fi
#cppcheck --library=std --check-level=exhaustive --inconclusive --enable=all --suppress=missingIncludeSystem --safety --checkers-report=error.txt --std=c++20 -v $1

cppcheck -I/Users/bill/Projects/src/Graphics/Libs/glm --library=std --suppress=missingIncludeSystem --check-level=exhaustive --inconclusive --enable=all --safety --checkers-report=error.txt --std=c++20 -v $1
