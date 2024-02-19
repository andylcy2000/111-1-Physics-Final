from vpython import *
import math
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
origin_x = radius_of_earth * cos(latitude) * cos(longitude)
origin_y = radius_of_earth * sin(latitude)
origin_z = -radius_of_earth * cos(latitude) * sin(longitude)

drag_force_constant = 0.5 #拖曳力常數

center_of_earth = vector(0,0,0) #地球球心
rocket = vector(origin_x,origin_y,origin_z) #火箭座標
theta = pi/2#火箭發射仰角
deviated_angle=0#火箭發射與北極偏角
direction = norm(vector(0,1/sin(latitude)*radius_of_earth,0)-rocket)#切線方向
#magd=mag(direction)
tanvl=sin(theta)/cos(theta)
tanvd=sin(deviated_angle)/cos(deviated_angle)
nr=norm(rocket)
y=vector(0,1,0)
dv=norm(cross(y,rocket))
add_v=direction+nr*tanvl+dv*tanvd
direction=norm(add_v)#火箭發射方向
print(direction)
v = direction
angular_velocity = 2*pi/86400 #地球自轉角速度
h = 0 #火箭離地表高度
omega = vector(0,angular_velocity,0)#要換成向量

g = 9.8
scene = canvas(width=500, height=500, center=vec(0, -0.2, 0), background=vec(0.8,0.8,0.8),align="left")
earth=sphere(pos=vec(0,0,0),radius=radius_of_earth, texture={'file':textures.earth})
sim=sphere(pos=rocket,radius=0.01*radius_of_earth,make_trail=True)#標示起始發射點
scene.center=rocket
'''
rocket_3D=cylinder(radius=radius,pos=rocket,color=color.cyan,)
rocket_3D.axis=norm(v)*length_of_missile_body
rocket_head=cone(radius=radius,pos=rocket+norm(v)*length_of_missile_body,color=color.white)
rocket_head.axis = norm(v)*length_of_warhead
'''
def mass(t):
    if t < acceleration_time:
        return original_mass-1025*t/acceleration_time
    else:
        return 475
def rho(h):
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
def push(direction):
    push = pushing_force*norm(direction)
    return push

def lift(rho,vec):
    return rho*volume*g*norm(vec)

def drag_force(rho,v):
    return -0.5*rho*drag_force_constant*mag2(v)*cross_section*norm(v)

def Coriolis_force(mass,omega,v):
    return -2*mass*cross(omega,v)

def gravity(mass,h,vec):
    return - gravitational_constant*mass_of_earth*mass/(h+radius_of_earth)**2*norm(vec)
t = 0
dt = 0.005
c_t=0
distance_traveled=0
projected_distance=0
while mag(rocket-center_of_earth)-radius_of_earth >= -100:
    rate(10000)
    m = mass(t)
    h = mag(rocket)-radius_of_earth
    RHO = rho(h)
    F = lift(RHO,rocket) + drag_force(RHO,v) + Coriolis_force(m,omega,v) + gravity(m,h,rocket)
    
    if t < acceleration_time: F += push(v)
    a = F/m
    v += a * dt
    rocket += v * dt
    distance_traveled+=mag(v*dt)#ds
    cos_phi=dot(rocket,v*dt)/(mag(rocket)*mag(v*dt))
    sin_phi=math.sqrt(1-cos_phi**2)
    projected_distance+=sin_phi*mag(v*dt)
    sim.pos=rocket
    scene.center=rocket
    if c_t>=10:
        print(t,rocket,mag(rocket-center_of_earth)-radius_of_earth,RHO)
        c_t=0
    t += dt
    c_t+=dt
'''
    rocket_3D.pos=rocket
    rocket_3D.axis=norm(v)*length_of_missile_body
    rocket_head.pos=rocket+norm(v)*length_of_missile_body
    rocket_head.axis = norm(v)*length_of_warhead
    # scene.center =rocket_3D
'''
    
# scene.center = rocket
print(acceleration_time)
print("time:",t)
print("distance traveled(km): ",round(distance_traveled/1000,2))#總射程
print("distance traveled(projected on the surface of earth)(km): ",round(projected_distance/1000,2))
print("original position: ",(origin_x,origin_y,origin_z))
print("final position: ",rocket)