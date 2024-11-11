import cv2 as c
import numpy as np
import pandas as pd
import math
import plotly.graph_objects as go
import plotly.express as px


def avoidancePlotter():
    # get the csv files into a dataframe
    df_velocity = pd.read_csv('../python-app-image-tof-logger/Velocity.csv')
    df_PoC = pd.read_csv('../python-app-image-tof-logger/PoCStorage.csv')
    # delete the unnamed column
    df_velocity = df_velocity.drop(df_velocity.columns[0], axis=1)
    df_PoC = df_PoC.drop(df_PoC.columns[0], axis=1)

    c.namedWindow('Avoidance', c.WINDOW_NORMAL)
    c.resizeWindow("Avoidance", 1000, 1000)

    # built up frames for cv2.imshow
    for i in range(df_velocity.shape[0]):
        # get the velocity and PoCStorage for each frame
        velocity = df_velocity.iloc[i, :].values
        PoC = df_PoC.iloc[i, :].values
        #if velocity[0] == 0 and velocity[1] == 0:
            #continue
        # built up the image
        imgsize = 120
        img = np.zeros((imgsize, imgsize, 3), np.uint8)
        center = int(imgsize/2)
        # draw the velocity
        img = c.arrowedLine(img, (center, center), (center + int(velocity[0]*40), center - int(velocity[1]*40)), (255, 0, 0), 2)
        maxPoC = np.argmax(PoC)
        for j in range(360):
            X = math.cos(math.radians(j))*50
            Y = math.sin(math.radians(j))*50
            sector = int(j/11.25)
            PoCValue = PoC[sector]-0.5
            color = [0,0,0]
            if PoCValue < 0:
                if PoCValue == -1.5:
                    color = [0,255,0]
                else:
                    color = [0,0,0]
            elif sector == maxPoC:
                color = [0,0,255]
            else:
                color = [510*PoCValue,510*PoCValue,510*PoCValue]
            img[int(center-Y),int(center+X)]  = color
        # show image in a window
        c.imshow('Avoidance', img)
        if i == 0:
            c.waitKey(10000)
        else:
            c.waitKey(50)

    c.destroyAllWindows()

def focusPlotter():
    df_Object = pd.read_csv('../python-app-image-tof-logger/FocusObjects.csv')
    df_Col = pd.read_csv('../python-app-image-tof-logger/FocusCenterCols.csv')

    # delete the unnamed column
    df_Object = df_Object.drop(df_Object.columns[0], axis=1)
    df_Col = df_Col.drop(df_Col.columns[0], axis=1)

    c.namedWindow('Focus', c.WINDOW_NORMAL)
    c.resizeWindow("Focus", 1000, 1000)

    for i in range(df_Object.shape[0]):
        imgsize = 8
        # get the velocity and PoCStorage for each frame
        img = np.zeros((imgsize, imgsize, 3), np.uint8)
        # draw the velocity
        object = df_Object.iloc[i, :].values
        Col = df_Col.iloc[i,:].values
        for idx,j in enumerate(object):
            if j == 1:
                if idx%8==Col:
                    img[int(idx/8),idx%8] = [0,0,255]
                else:
                    img[int(idx/8),idx%8] = [0,0,0]
            else:
                img[int(idx/8),idx%8] = [255,255,255]
        # show image in a window
        c.imshow('Focus', img)
        c.waitKey(10)


    c.destroyAllWindows()


def positionPlotter():
    df_Position = pd.read_csv('../python-app-image-tof-logger/Position.csv')
    df_Position = df_Position.drop(df_Position.columns[0], axis=1)

    c.namedWindow('Position', c.WINDOW_NORMAL)
    c.resizeWindow("Position", 1000, 1000)

    imgsize = 100
    # get the velocity and PoCStorage for each frame
    img = np.zeros((imgsize, imgsize, 3), np.uint8)

    for i in range(df_Position.shape[0]):
        center = int(imgsize/2)-1
        # draw the velocity
        x= int(df_Position.iloc[i, :].values[0])
        y= int(df_Position.iloc[i, :].values[1])
        if x != 0 and y != 0:
            # make cross
            img[center-y,center+x] = [0,0,255]
            img[center-y,center+x+1] = [0,0,255]
            img[center-y,center+x-1] = [0,0,255]
            img[center-y+1,center+x] = [0,0,255]
            img[center-y-1,center+x] = [0,0,255]

        if i%4==3:
            # add lines to the image from the center to the border with angles [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5]
            # FRONT
            yaw = df_Position.iloc[i, :].values[2]
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 22.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 22.5)))), (255, 0, 0), 1)
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw - 22.5))),
                         int(center - 45 * math.sin(math.radians(yaw - 22.5)))), (255, 0, 0), 1)
            # OTHERS
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 67.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 67.5)))), (0, 255, 0), 1)
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 112.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 112.5)))), (0, 255, 0), 1)

            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 157.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 157.5)))), (0, 255, 0), 1)
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 202.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 202.5)))), (0, 255, 0), 1)

            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 247.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 247.5)))), (0, 255, 0), 1)
            img = c.line(img, (center, center), (int(center + 45 * math.cos(math.radians(yaw + 292.5))),
                         int(center - 45 * math.sin(math.radians(yaw + 292.5)))), (0, 255, 0), 1)

            c.imshow('Position', img)
            c.waitKey(20)
            img = np.zeros((imgsize, imgsize, 3), np.uint8)

def viconPlotter():
    df = pd.read_csv('../../data/.csv')
    firstRow = pd.DataFrame(df.columns).T
    firstRow.columns = firstRow.iloc[0]
    df = pd.concat([firstRow,df],axis=0).reset_index(drop=True)
    df.columns = ['timestamp', 'object', 'value']
    #df_Position = df_Position.drop(df_Position.columns[0], axis=1)
    #df_Position.drop(df_Position.tail(1000).index,inplace=True)
    df_drone_pos_x = df[df['object'] == "drone_lukas_posx"].reset_index(drop=True)
    df_drone_pos_y = df[df['object'] == "drone_lukas_posy"].reset_index(drop=True)
    df_drone_pos_z = df[df['object'] == "drone_lukas_posz"].reset_index(drop=True)
    df_person_pos_x = df[df['object'] == "helmet_lukas_posx"].reset_index(drop=True)
    df_person_pos_y = df[df['object'] == "helmet_lukas_posy"].reset_index(drop=True)


    c.namedWindow('Position', c.WINDOW_NORMAL)
    c.resizeWindow("Position", 1000, 1000)

    imgsize = 500
    # get the velocity and PoCStorage for each frame
    img = np.zeros((imgsize, imgsize, 3), np.uint8)

    index = np.arange(imgsize)

    person_index =0
    start_index = 0
    end_index = 1000000
    mode = 1
    distance_counter = 0
    press_counter = 0
    mean_distance = 0

    person_update = False

    init = False
    last_x = 0
    last_y = 0

    for i in range(df_drone_pos_x.shape[0]):
        timestamp = float(df_drone_pos_x.iloc[i][0])
        if person_index < df_person_pos_x.shape[0]:
            if timestamp>df_person_pos_x.iloc[person_index][0]:
                person_update = True
        if i >= start_index and i < end_index:
            center = int(imgsize / 2) - 1
            # draw the velocity
            if df_drone_pos_z.iloc[i][2] >= 0.5:
                x = int(float(df_drone_pos_x.iloc[i][2])*100)
                y = int(float(df_drone_pos_y.iloc[i][2])*100)
                # make drone
                # center
                c.rectangle(img, (center + x - 10,center - y - 10), (center + x + 10,center - y + 10), (0, 0, 255), c.FILLED)
                # propellors
                c.line(img, (center + x,center - y), (center + x + 15,center - y + 15), (0, 0, 255), 5)
                c.line(img, (center + x,center - y), (center + x - 15,center - y + 15), (0, 0, 255), 5)
                c.line(img, (center + x,center - y), (center + x + 15,center - y - 15), (0, 0, 255), 5)
                c.line(img, (center + x,center - y), (center + x - 15,center - y - 15), (0, 0, 255), 5)

                if person_update:
                # make person
                    x = int(float(df_person_pos_x.iloc[person_index][2])*100)
                    y = int(float(df_person_pos_y.iloc[person_index][2])*100)

                    if not init:
                        last_x = x
                        last_y = y
                        init = True
                    else:
                        distance_counter += 1
                        mean_distance += math.sqrt((x-last_x)**2+(y-last_y)**2)
                        last_x = x
                        last_y = y

                    # head
                    c.ellipse(img,(center +x,center-y),(6,6),0,0,360,(0,255,0),c.FILLED)
                    # body
                    c.ellipse(img,(center +x,center-y+12),(3,10),0,0,360,(0,255,0),c.FILLED)
                    # arms
                    c.ellipse(img,(center + x + 3,center-y+12),(2,5),330,0,360,(0,255,0),c.FILLED)
                    c.ellipse(img,(center + x - 3,center-y+12),(2,5),30,0,360,(0,255,0),c.FILLED)
                    # legs
                    c.ellipse(img,(center + x + 3,center-y+22),(2,5),330,0,360,(0,255,0),c.FILLED)
                    c.ellipse(img,(center + x - 3,center-y+22),(2,5),30,0,360,(0,255,0),c.FILLED)

                #make obstacles

                c.rectangle(img, (center + 200,center +200), (center + 100,center + 100), (255, 255, 255), 4)
                c.rectangle(img, (center + 200,center -200), (center + 100,center - 100), (255, 255, 255), 4)
                c.rectangle(img, (center - 200,center +200), (center - 100,center + 100), (255, 255, 255), 4)
                c.rectangle(img, (center - 200,center -200), (center - 100,center - 100), (255, 255, 255), 4)

                # make boarder
                c.rectangle(img, (2,2), (imgsize-2,imgsize-2), (255, 255, 255), 4)


                c.imshow('Position', img)
                if mode == 0:
                    if press_counter == 50:
                        c.waitKey(50000)
                        print(f"index: {i}")
                        press_counter = 0
                    else:
                        c.waitKey(20)
                        press_counter += 1
                else:
                    c.waitKey(20)
                img = np.zeros((imgsize, imgsize, 3), np.uint8)

        if person_update:
            person_index += 1
            person_update = False
    print(mean_distance/(distance_counter)*0.5)

def plot1():
    results = ['avoided', 'avoided', 'avoided', 'avoided', 'crashed', 'avoided', 'crashed', 'avoided', 'avoided',
                    'crashed', 'avoided']
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=[1.42,1.68,1.01,2.01,2.78,1.85,2.52,1.86,1.92,2.51,2.18],
        y=results,
        marker=dict(color="black", size=25),
        mode="markers"
    ))
    fig.update_layout(
        font=dict(
            family="Courier New, monospace",
            size=30,  # Set the font size here
        )
    )
    fig.update_layout(title="Avoidance Test Results 1",
                      xaxis_title="Approaching Speed (m/s)",
                      yaxis_title="Result")
    fig.add_vrect(
        x0=0.8,
        x1=2.2,
        fillcolor="green",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=2.2,
        x1=2.49,
        fillcolor="orange",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=2.49,
        x1=3.0,
        fillcolor="red",
        opacity=0.2,
        line_width=0,
    )
    fig.show()

def plot2():
    results = ['avoided', 'avoided', 'avoided', 'crashed', 'avoided', 'avoided', 'avoided', 'crashed', 'avoided',
                    'crashed']
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=[1.4,1.93,0.8,2.32,1.75,1.78,1.7,2.22,2.03,2.6],
        y=results,
        marker=dict(color="black", size=25),
        mode="markers"
    ))
    fig.update_layout(
        font=dict(
            family="Courier New, monospace",
            size=30,  # Set the font size here
        )
    )
    fig.update_layout(title="Avoidance Test Results 2",
                      xaxis_title="Approaching Speed (m/s)",
                      yaxis_title="Result")
    fig.add_vrect(
        x0=0.6,
        x1=2.05,
        fillcolor="green",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=2.05,
        x1=2.2,
        fillcolor="orange",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=2.2,
        x1=3.0,
        fillcolor="red",
        opacity=0.2,
        line_width=0,
    )
    fig.show()

def plot3():
    results = ['avoided', 'crashed', 'avoided', 'avoided', 'crashed', 'avoided', 'avoided', 'avoided', 'avoided',
                    'avoided']
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=[1.28,1.63,1.23,1.63,1.91,1.41,1.5,1.69,1.88,1.89],
        y=results,
        marker=dict(color="black", size=25),
        mode="markers"
    ))
    fig.update_layout(
        font=dict(
            family="Courier New, monospace",
            size=30,  # Set the font size here
        )
    )
    fig.update_layout(title="Avoidance Test Results 3",
                      xaxis_title="Approaching Speed (m/s)",
                      yaxis_title="Result")
    fig.add_vrect(
        x0=1,
        x1=1.89,
        fillcolor="green",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=1.89,
        x1=1.91,
        fillcolor="orange",
        opacity=0.2,
        line_width=0,
    )
    fig.add_vrect(
        x0=1.91,
        x1=2.0,
        fillcolor="red",
        opacity=0.2,
        line_width=0,
    )
    fig.show()



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #avoidancePlotter()
    #focusPlotter()
    #positionPlotter()
    viconPlotter()
    #plot1()
    #plot2()
    #plot3()