tmux kill-session
sleep 1
tmux new-session -d
tmux set -g mouse on
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux select-layout even-horizontal

tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "cd ~/Desktop/TurtleBotPNU" C-m
tmux send "roslaunch pnu_tb3_launch followbot_core_2.launch" C-m
sleep 1
tmux select-pane -t 1
tmux send "source ~/.bashrc" C-m
tmux send "cd ~/Desktop/TurtleBotPNU" C-m
tmux send "roslaunch realsense2_camera rs_camera.launch" C-m
tmux attach-session -d
