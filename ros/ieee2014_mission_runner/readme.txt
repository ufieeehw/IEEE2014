def go_to_point(point):
    start controller
    send point to controller
    move to block position
    wait until near point
    stop controller

wait for starting signal
start looking for blocks (enable camera, pan)
wait for block positions
disable camera
for each block:
    enable targetting solver
    go_to_point(block position)
    switch to camera targetting
    fire
    wait remainder of 3 seconds
go_to_point(ending zone)
