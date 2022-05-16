brickrun -r -- ~/shuffler ${1+"$@"} 2>&1 | tee run-$(date "+%Y-%m-%d_%H:%M:%S").log
