suffix=$(date "+%Y-%m-%d_%H:%M:%S")
brickrun -r -- ~/shuffler -o run-${suffix}.json ${1+"$@"} 2>&1 | tee run-${suffix}.log
