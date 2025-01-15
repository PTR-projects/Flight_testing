import matplotlib.pyplot as plt
import csv
import os


# Plotting variables
time = []
alt = []
accX = []
state = []



# File system
fileList = os.listdir()
csvList = []

start_time = 0
cnt = 0
with open("data/output.csv", 'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        if cnt == 0:
            start_time = float(row[0])
            
        cnt = cnt + 1
        if cnt > 5:
            time.append(float(row[0]) - start_time)
            accX.append(float(row[1]))
            alt.append(float(row[2]))
            state.append(float(row[3]))

    


#plt.style.use('dark_background')

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

ax1.set_xlabel('Time [ms]')
ax1.grid(True)

ax1.set_ylabel('Altitude', color='y')
ax1.tick_params(axis='y', labelcolor='y')
#ax1.fill_between(x, F, 0, alpha=0.2, color='y')

ax2.set_ylabel('State', color='b')  # we already handled the x-label with ax1
ax2.tick_params(axis='y', labelcolor='b')

ax1.plot(time, accX, color='y')
ax2.plot(time, state, color='b')


fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()