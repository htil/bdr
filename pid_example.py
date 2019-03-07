SAMPLETIME = 0.5
KP = 0.02
KD = 0.01
KI = 0.005

prev_error = 0
sum_error = 0

while True:
    error = le.GetError(img)[1]#getlineerror

    new_vel += (error * KP) + (prev_error * KD) + (sum_error * KI)
    # send new vel

    sleep(SAMPLETIME)

    prev_error = error
    sum_error += error
