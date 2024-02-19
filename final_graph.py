from vpython import *
import math
import cv2

original_mass = 1500 #飛彈原本質量
radius = 10000*0.23 #飛彈半徑
length = 6.1 #飛彈長度
length_of_warhead = 10000*length*3/14.4 #彈頭長度
length_of_missile_body = 10000*length*11.4/14.4 #飛彈本體長度
cross_section = radius**2*pi/100000000 #飛彈橫截面
volume = 1/3*(cross_section*length_of_warhead+cross_section*length_of_missile_body)/10000 #飛彈體積
pushing_force = 1814*1000*(275/2000)**(5/3) #推進力量值
acceleration_time = 1025*9.8*330/pushing_force #加速時間

mass_of_earth = 5.9722*10**24 #地球質量
radius_of_earth = 6.371 * 10**6 #地球半徑
gravitational_constant = 6.6743 * 10 **(-11) #重力常數
longitude = pi*121/180#東經
latitude = pi*23.5/180#北緯

#座標系
origin_x = radius_of_earth * cos(latitude) * cos(longitude)
origin_y = radius_of_earth * sin(latitude)
origin_z = -radius_of_earth * cos(latitude) * sin(longitude)

drag_force_constant = 0.5 #拖曳力常數

center_of_earth = vector(0,0,0) #地球球心
theta = 76.5*pi/180#火箭發射仰角
omega = vector(0,2*pi/86400,0) #地球自轉角速度
h = 0 #火箭離地表高度

g = 9.8#重力


img = cv2.imread('taiwan_map.jpg')#讀取台灣地圖
rows,cols,ch = img.shape #寬671km，長799km，附近經度1度 = 101km，附近緯度1度 = 111km

def mass(t):#燃料燃燒，質量隨時間降低
    if t < acceleration_time:
        return original_mass-1025*t/acceleration_time
    else:#燃燒停止
        return 475

def rho(h):#計算空氣密度
    if h > 25000:
        T = -131.21 + 0.00299*h
        p = 2.488 * ((T+273.1)/216.6)**(-11.388)
    elif 25000 >= h > 11000:
        T = -56.46
        p = 22.65*e**(1.73-0.000157*h)
    else:
        T = 15.04-0.00649*h
        p = 101.29*((T+273.1)/288.08)**(5.256)
    rho = p/(0.2869*(T+273.1))
    return rho

def push(direction):#計算火箭推力
    push = pushing_force*norm(direction)
    return push

def lift(rho,vec):#計算浮力
    return rho*volume*g*norm(vec)

def drag_force(rho,v):#計算拖曳力
    return -0.5*rho*drag_force_constant*mag2(v)*cross_section*norm(v)

def Coriolis_force(mass,omega,v):#計算科氏力
    return -2*mass*cross(omega,v)

def gravity(mass,h,vec):#計算重力
    return - gravitational_constant*mass_of_earth*mass/(h+radius_of_earth)**2*norm(vec)

def trans(f_vec):#座標轉換成經緯度
    zero_vec=vector(1,0,0)
    new_vec=vector(f_vec.x,0,f_vec.z)
    cos_angle=dot(new_vec,zero_vec)/(mag(new_vec)*mag(zero_vec))
    latitude=asin(f_vec.y/radius_of_earth)*180/pi
    if f_vec.z<0:
        longitude=acos(cos_angle)*180/pi
        return "Eastern", longitude, latitude
    else:
        longitude=acos(cos_angle)*180/pi
        return "Western", longitude, latitude

deviated_angle=0#火箭發射方向，從X軸向逆時針旋轉
while deviated_angle < 360/180*pi:
    rocket = vector(origin_x,origin_y,origin_z) #火箭座標
    theta = 76.5*pi/180#火箭發射仰角

    #計算火箭發射位置的坐標系
    x =  norm(cross(vector(0,1/sin(latitude)*radius_of_earth,0),rocket))
    y =  norm(cross(rocket,x))
    z = norm(cross(x,y))
    v = norm(norm(cos(deviated_angle)*x+sin(deviated_angle)*y)+z*tan(theta))#火箭發射方向
    h = 0 #火箭離地表高度

    t = 0
    dt = 0.005
    distance_traveled=0#火箭移動距離
    projected_distance=0#火箭投影地球的移動距離
    while mag(rocket-center_of_earth)-radius_of_earth >= -100:#火箭插入地球之前
        rate(10000)
        m = mass(t)#計算質量
        h = mag(rocket)-radius_of_earth#計算離地高度
        RHO = rho(h)#計算空氣密度
        F = lift(RHO,rocket) + drag_force(RHO,v) + Coriolis_force(m,omega,v) + gravity(m,h,rocket)#計算火箭受力
        if t < acceleration_time: F += push(v)#燃料燃燒殆盡前仍有推力
        a = F/m
        v += a * dt
        rocket += v * dt
        distance_traveled+=mag(v*dt)#ds
        # 計算投影
        cos_phi=dot(rocket,v*dt)/(mag(rocket)*mag(v*dt))
        sin_phi=math.sqrt(1-cos_phi**2)
        projected_distance+=sin_phi*mag(v*dt)
        scene.center=rocket
        t += dt
    we,new_long,new_lat=trans(rocket)#東西半球、經度、緯度
    position = (int(img.shape[1]//2+(new_long-121)*101/671*img.shape[1]),int(img.shape[0]//2-(new_lat-23.5)*111/799*img.shape[0]))#計算地圖上的位置
    cv2.circle(img,position,3,(0,0,255),thickness=-1)#標記火箭落點
    deviated_angle += 1*pi/180#改變發射方向
    print(deviated_angle)

cv2.namedWindow('image')
cv2.imshow('image',img)#顯示落點圖
cv2.waitKey(0)
