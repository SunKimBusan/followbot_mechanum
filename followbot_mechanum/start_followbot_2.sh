tmux kill-session
sleep 1
tmux new-session -d
tmux set -g mouse on
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -R
tmux select-layout even-horizontal
# tmux split-window -h
# tmux select-pane -R

tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
tmux send "roslaunch followbot_mechanum followbot_mechanum.launch" C-m
sleep 1
tmux select-pane -t 1
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
tmux send "rosrun followbot_mechanum depth_location_3.py" C-m
sleep 1
tmux select-pane -t 2
tmux send "source ~/.bashrc" C-m
tmux send "roscd followbot_mechanum" C-m
#tmux send "rosrun followbot_mechanum controller_ver4.py" C-m

tmux attach-session -d
