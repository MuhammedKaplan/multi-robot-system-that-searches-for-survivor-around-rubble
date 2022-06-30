# =============================================================================
# Kütüphaneler
# =============================================================================
import math, collections, struct
import numpy as np
import sys
from controller import Robot, Motor, DistanceSensor, PositionSensor, Compass, InertialUnit, Emitter, Receiver, Supervisor


# =============================================================================
# Her bir 8x8 grid haritasını arayacak fonksiyon. Bu fonksiyonda yol bulma 
# algoritmaları bulunmaktadır. Ayrıca kazazede ve engel tespiti bu 
# fonksiyonda yapılmaktadır.
# =============================================================================

def grid_search(robot, new_x, new_y):
    grid_search.start_time = robot.getTime()
    
    grid_search.robot = robot
    
    # Define the time step 
    grid_search.time_step = 64
    
    target_pos_search.final_wait = 0 
    target_pos_search.final_wait_state = 0
    
    # Define max motor speed
    grid_search.max_speed = 6
    
    search_area.found = 0
    
    
    grid_search.wall = 3
    grid_search.visit = 1
    grid_search.goal = 0  
    
    # Tüm haritaları tuttuğumuz dizinin hangi listesinde olduğumuzun tespiti
    numberOfArea = (search_area.cur_x + 5 * search_area.cur_y)
    
    # 5x5 lik haritada nerede bulunduğumuza göre harita oluşturumu
    grid_search.grid_map = grid_search.grid_map_all[numberOfArea]
    
    # Başlangıç veya bir sonraki grid ile gelen yeni x, y değerleri
    grid_search.cur_pos_x = new_x
    grid_search.cur_pos_y = new_y
    
    grid_search.target_temp_pos_seq = 0
    
    # Robotun bir ilerleme adım miktarı
    grid_search.increment_range = 12.464
    grid_search.movement_check = 0
    grid_search.extend_check = 0
    
    block_check_call = 0
    
    grid_search.rot_angle = 0
    grid_search.rot_state = 0
    
    grid_search.tumble_state = 0
    
    grid_search.final_state = 0
    grid_search.exit_flag = 0
    
    receive_message.south_check = 1
    
    # Motor tanımlamaları
    grid_search.leftF_motor = robot.getDevice('leftF')
    grid_search.leftR_motor = robot.getDevice('leftR')
    grid_search.rightF_motor = robot.getDevice('rightF')
    grid_search.rightR_motor = robot.getDevice('rightR')
    
    # Başlangıç motor pozisyonları
    grid_search.leftF_motor.setPosition(0)
    grid_search.leftR_motor.setPosition(0)
    grid_search.rightF_motor.setPosition(0)
    grid_search.rightR_motor.setPosition(0)
    
    # Başlangıç hızları
    grid_search.leftF_motor.setVelocity(0)
    grid_search.leftR_motor.setVelocity(0)
    grid_search.rightF_motor.setVelocity(0)
    grid_search.rightR_motor.setVelocity(0)
    
    # Mesafe sensörlerinin tanımlanması ve aktifleştirilmesi
    ds_fall1 = robot.getDevice('ds_fall1')
    ds_fall1.enable(grid_search.time_step)
    
    ds_fall2 = robot.getDevice('ds_fall2')
    ds_fall2.enable(grid_search.time_step)
    
    ds_front = []
    dsFrontNames = [
        'ds_front0', 'ds_front1', 'ds_front2', 
        'ds_front3', 'ds_front4', 'ds_front5', 
        'ds_front6', 'ds_front7', 'ds_front8', 
        'ds_front9', 'ds_front10', 'ds_front11', 
        'ds_front12', 'ds_front13', 
        ]
    
    ds_right = []
    dsRightNames = [
        'ds_right', 'ds_right1', 'ds_right2', 
        'ds_right3', 'ds_right4', 
        ]
    
    ds_left = []
    dsLeftNames = [
        'ds_left', 'ds_left1', 'ds_left2', 
        'ds_left3', 'ds_left4', 
        ]
    
    for i in range(14):
        ds_front.append(robot.getDevice(dsFrontNames[i]))
        ds_front[i].enable(grid_search.time_step)
    
    for i in range(5):
        ds_right.append(robot.getDevice(dsRightNames[i]))
        ds_right[i].enable(grid_search.time_step)
        
    for i in range(5):
        ds_left.append(robot.getDevice(dsLeftNames[i]))
        ds_left[i].enable(grid_search.time_step)
    
    # Positional sensörlerinin tanımlanması ve aktif edilmesi
    ps_left = robot.getDevice('ps_left')
    ps_left.enable(grid_search.time_step)
    
    ps_right = robot.getDevice('ps_right')
    ps_right.enable(grid_search.time_step)
    
    # Pusulanın tanımlanması ve aktif edilmesi
    compass = robot.getDevice('compass')
    compass.enable(grid_search.time_step)
    
    # IMU sensörünün tanımlanması ve aktif edilmesi
    imu = robot.getDevice('iu')
    imu.enable(grid_search.time_step)   
    
    # Haberleşme birimlerinin tanımlanması ve aktifleştirilmesi
    grid_search.emitter = robot.getDevice('emitter')
    
    grid_search.receiver = robot.getDevice('receiver')
    grid_search.receiver.enable(grid_search.time_step)
    
    
    if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] not in (search_area.r1,search_area.r2,search_area.r3)):
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = search_area.rSelf
        # draw_map(grid_search.grid_map)
    
    # İlk konumun diğer robotlara iletilmesi
    message = struct.pack("hhhhh",search_area.rSelf,numberOfArea,1,grid_search.cur_pos_x, grid_search.cur_pos_y)
    byte = 5
    byte_message = struct.pack("h",byte)
    grid_search.emitter.send(byte_message)
    grid_search.emitter.send(message)
    
    # Ana döngü:
    while (robot.step(grid_search.time_step) != -1 and grid_search.exit_flag != 1):
        
        # Robotun mevcut konumu
        robot_pos = robot_trans_field.getSFVec3f()
        
        # Kazazede tespit kontrolleri
        if(search_area.s_pos[0] - 0.77<robot_pos[0]<search_area.s_pos[0] + 0.77 and 
            search_area.s_pos[2] - 1.13<robot_pos[2]<search_area.s_pos[2] + 1.13 and 
            search_area.s_found == 0):
            print("survivor")
            print("robot cur::",grid_search.cur_pos_x, grid_search.cur_pos_y)
            print("search area cur::", search_area.cur_x, search_area.cur_y)
            search_area.s_found = 1
            
        if(search_area.s2_pos[0] - 0.77<robot_pos[0]<search_area.s2_pos[0] + 0.77 and 
            search_area.s2_pos[2] - 1.13<robot_pos[2]<search_area.s2_pos[2] + 1.13 and
            search_area.s2_found == 0):
            print("survivor2")
            print("robot cur::",grid_search.cur_pos_x, grid_search.cur_pos_y)
            print("search area cur::", search_area.cur_x, search_area.cur_y)
            search_area.s2_found = 1
                        
        if(search_area.s3_pos[0] - 0.77<robot_pos[0]<search_area.s3_pos[0] + 0.77 and 
            search_area.s3_pos[2] - 1.13<robot_pos[2]<search_area.s3_pos[2] + 1.13 and
            search_area.s3_found == 0):
            print("survivor3")
            print("robot cur::",grid_search.cur_pos_x, grid_search.cur_pos_y)
            print("search area cur::", search_area.cur_x, search_area.cur_y)
            search_area.s3_found = 1
            
            
            
        
        # IMU sensör değerlerinin alınması
        grid_search.imu_value = imu.getRollPitchYaw()
        
        # Pusula değerlerinin alınması
        compass_value = compass.getValues()
        
        # Positional sensör verilerinin alınması
        grid_search.ps_left_value = ps_left.getValue()
        grid_search.ps_right_value = ps_right.getValue()        
        
        # Mesafe sensörlerinin değerlerinin alınması
        grid_search.ds_front_values = []
        grid_search.ds_right_values = []
        grid_search.ds_left_values = []
        
        for i in range(14):
            grid_search.ds_front_values.append(ds_front[i].getValue())
            
        for i in range(5):
            grid_search.ds_right_values.append(ds_right[i].getValue())
            
        for i in range(5):
            grid_search.ds_left_values.append(ds_left[i].getValue())
        
        if(rotation_direction.counter > 0):
            rotation_direction.counter -= 1
        
        # Robota kuzeyin bulunduğu açının hesaplanması
        grid_search.north = math.atan2(compass_value[0], compass_value[2])   
           
        # Harita güncelle
        update_map()
        
        # Diğer robotlardan gelen mesajları al
        receive_message()
        
        # Başlangıçta bir sefer olmak üzere engel kontrolü
        if(block_check_call == 0):
           block_check()
           block_check_call = 1
        
        # Bir sonraki gidilecek konumun bulunması
        target_pos_search()        
        
        # Bir sonraki konuma gidilmeyi gerçekleştiren fonk.
        cal_next_step_range()        
        
        # Robotun belirlenmiş yöne dönüp dönmediğinin kontrolü
        rotate_check()        
        
        # Takla kontrolü
        tumble_check()
        
        # Robotun hedef konuma varıp varmadığının kontrolü
        rotate_pos_check()
        
        # Motor Pozisyonlarının atanması
        grid_search.leftF_motor.setPosition(cal_next_step_range.rangeLeft)
        grid_search.leftR_motor.setPosition(cal_next_step_range.rangeLeft)
        grid_search.rightF_motor.setPosition(cal_next_step_range.rangeRight)
        grid_search.rightR_motor.setPosition(cal_next_step_range.rangeRight)
        
        # Hızların atanması
        set_speed(grid_search.left_speed, grid_search.right_speed)        
        
        
        pass

def set_speed(left_speed, right_speed):
# =============================================================================
# Bu fonksiyonda hız atamaları gerçekleştirilir
# =============================================================================
    
    grid_search.leftF_motor.setVelocity(left_speed)
    grid_search.leftR_motor.setVelocity(left_speed)
    grid_search.rightF_motor.setVelocity(right_speed)
    grid_search.rightR_motor.setVelocity(right_speed)

def draw_map(print_map):
# =============================================================================
# Bu fonksiyonda gönderilen liste uzunluğuna uygun bir şekilde konsola
# bastırılır.
# =============================================================================
    # Print updated map
    rows = len(print_map)
    
    for i in range(rows - 1, -1, -1):
        print(print_map[i])
    print("***"*rows)
        
def update_map():
# =============================================================================
# Bu fonksiyon değişen durumlara göre haritayı günceller
# =============================================================================
        
    # If robot discover undiscovered area then update map
    if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] == 0
       or grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] == 1):
    
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = search_area.rSelf
        
        # draw_map(grid_search.grid_map)    

def receive_message():
# Bu fonksiyon diğer robotlardan gelen mesajları alır ve türüne göre
# değerlendirerek harita güncellemesi sağlar veya karşı cevap üretir.
    draw = 0
    # Gelen mesaj kuyruğu dolu olduğu sürece
    while grid_search.receiver.getQueueLength()>0:     
        # Mesajın türünü belirten ilk mesajı al
        comming_message_byte = grid_search.receiver.getData()
        # Mesajı aç
        message_byte = struct.unpack("h",comming_message_byte)
        # Bir sonraki pakete geç
        grid_search.receiver.nextPacket()      
        
        # Gönderilen mesaj türü -1
        if message_byte[0] == -1:
            # Bir sonraki mesajı al
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("h",comming_message)
            
            # Bu mesaj türü diğer robotların grid aramasını bitirip bitirmediğini bildirir
            if dataList[0] == search_area.r1: 
                search_area.r5_final = 1  
                # print("r5 :",search_area.r5_final)
            if dataList[0] == search_area.r2: 
                search_area.r6_final = 1
                # print("r6 :",search_area.r6_final)
            if dataList[0] == search_area.r3: 
                search_area.r7_final = 1
                # print("r7 :",search_area.r7_final)
        
        # Bu mesaj türü robotların grid geçişlerinin engel bilgisini bildirir
        elif message_byte[0] == -2:
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("hh",comming_message)
            if(dataList[0] == 0):
                grid_search.next_grid_west[0][dataList[1]] = 3
            elif(dataList[0] == 1):
                grid_search.next_grid_north[0][dataList[1]] = 3      
            elif(dataList[0] == 2):
                grid_search.next_grid_east[0][dataList[1]] = 3
            elif(dataList[0] == 3):
                grid_search.next_grid_south[0][dataList[1]] = 3
        
        # Bu mesaj türü robotların birbirlerinin hedef konumları için onay istediğini bildirir.
        # Belirlenen koşullara göre onay verilir ya da verilmez
        elif(message_byte[0] == -3):
            numberOfArea = search_area.cur_x + 5 * search_area.cur_y
            
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("hhhhf",comming_message)
            
            ok = 1
            # Eğer aynı alandalar ise
            if numberOfArea == dataList[1]:
                # Gelen hedef bizim hedefimizle uyuşuyor mu
                if(dataList[2] == target_pos_search.x and dataList[3] == target_pos_search.y):
                    # Hedefi kim önce belirlediyse üstünlük ondadır
                    if(target_pos_search.target_time < dataList[4]):
                        ok = 0
                        target_pos_search.conf = 1
                    
                    # Eğer aynı zamanda belirlenmişse robot kimlik numarası büyük olan üstündür
                    if(target_pos_search.target_time == dataList[4]):
                        if(dataList[0] < search_area.rSelf):
                            ok = 0
                            target_pos_search.conf = 1
                    
                    if ok == 1:   
                        grid_search.grid_map[target_pos_search.x][target_pos_search.y] = dataList[0]
                
                # Onay mesajının gönderimi
                byte = -4
        
                message = struct.pack("hhh",search_area.rSelf,dataList[0],ok)
                
                byte_message = struct.pack("h",byte)
                grid_search.emitter.send(byte_message)
                grid_search.emitter.send(message)
        
        # Eğer aynı alanda değillerse onayladığını belirten mesaj
            else:
                
                byte = -4
        
                message = struct.pack("hhh",search_area.rSelf,dataList[0],1)
                
                byte_message = struct.pack("h",byte)
                grid_search.emitter.send(byte_message)
                grid_search.emitter.send(message)
        
        # Onay mesajının alınması. Burada bütün robotlardan onay gelirse harekete geçilir.
        elif(message_byte[0] == -4):
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("hhh",comming_message)
            
            if dataList[1] == search_area.rSelf:
                
                if dataList[0] == search_area.r1:   
                    receive_message.r1_confirm = dataList[2]
                    
                if dataList[0] == search_area.r2:   
                    receive_message.r2_confirm = dataList[2]
                    
                if dataList[0] == search_area.r3:   
                    receive_message.r3_confirm = dataList[2]
                
                if(receive_message.r1_confirm == 1 and receive_message.r2_confirm == 1 and 
                   receive_message.r3_confirm == 1):
                    
                    receive_message.r1_confirm = 0
                    receive_message.r2_confirm = 0
                    receive_message.r3_confirm = 0
                    
                    target_pos_search.conf = 1
                
                else:
                    target_pos_search.conf = 0
        
        # Harita bilgilerini gönderildiği mesaj türü(pozitif)
        # Burada ilk gönderilen mesajdan alınan sayı üzerinden
        # kaç tane verinin gönderildiği bilinir ve buna göre harita 
        # güncellenir.
        else:
            comming_message = grid_search.receiver.getData()
            dataList = struct.unpack("h"*message_byte[0],comming_message)
            
            lendatalist = len(dataList)
            if(grid_search.extend_check == 0):
                j = 2              
                while j < lendatalist:
                    if(dataList[j+1] > 7 or dataList[j+2] > 7 or 
                       dataList[j+1] < 0 or dataList[j+2] < 0):
                        
                        if j == lendatalist - 3:
                            dataList = dataList[:j]
                            break
                        else:
                            dataList = dataList[:j] + dataList[j+3:]
                            lendatalist -= 3
                            j -= 3
                    j += 3
            
            for i in range(2, len(dataList), 3):
                update_pos_val = dataList[i]
                update_pos_x = dataList[i+1]
                update_pos_y = dataList[i+2]
                
                if (search_area.cur_x + 5 * search_area.cur_y) == dataList[1]:
                    if(grid_search.grid_map[update_pos_x][update_pos_y] != search_area.rSelf):
                        if((grid_search.grid_map[update_pos_x][update_pos_y] == search_area.r1 and dataList[0] == search_area.r1)
                           or (grid_search.grid_map[update_pos_x][update_pos_y] == search_area.r2 and dataList[0] == search_area.r2)
                               or (grid_search.grid_map[update_pos_x][update_pos_y] == search_area.r3 and dataList[0] == search_area.r3)
                               or grid_search.grid_map[update_pos_x][update_pos_y] == 0
                               or grid_search.grid_map[update_pos_x][update_pos_y] == 1):
                            grid_search.grid_map[update_pos_x][update_pos_y] = update_pos_val
                
                elif(search_area.map_chance == 1):
                    grid_search.grid_map_all[dataList[1]][update_pos_x][update_pos_y] = update_pos_val
                
        grid_search.receiver.nextPacket()
        draw = 1


def send_message():
# =============================================================================
# Bu fonksiyon harita bilgilerinin gönderilmesini sağlar.
# =============================================================================
    if((grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 0
       or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 1) and
       target_pos_search.message_send == 1):
        
        grid_search.grid_map[target_pos_search.x][target_pos_search.y] = search_area.rSelf
    
    if grid_search.exit_flag == 1:
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
   
    byte = 5
    numberOfArea = search_area.cur_x + 5 * search_area.cur_y
    message = struct.pack("5h",search_area.rSelf, numberOfArea,grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y]
                          ,grid_search.cur_pos_x, grid_search.cur_pos_y)
    for j in range(len(grid_search.grid_map[0])):
        for i in range(len(grid_search.grid_map)):
            if(grid_search.grid_map[i][j] == 1 or grid_search.grid_map[i][j] == search_area.rSelf
               or grid_search.grid_map[i][j] == 3):
                message += struct.pack("hhh",grid_search.grid_map[i][j],i,j)
                byte += 3
        
    byte_message = struct.pack("h",byte)
    grid_search.emitter.send(byte_message)
    grid_search.emitter.send(message)
    target_pos_search.message_send = 0

def send_sync_message():
# =============================================================================
# Bu fonksiyon robotların hedefleri için onay istediği fonksiyon
# =============================================================================
    byte = -3
    numberOfArea = search_area.cur_x + 5 * search_area.cur_y
    message = struct.pack("hhhhf",search_area.rSelf,numberOfArea,target_pos_search.x,target_pos_search.y,target_pos_search.target_time)
    
    byte_message = struct.pack("h",byte)
    grid_search.emitter.send(byte_message)
    grid_search.emitter.send(message)
    
    target_pos_search.target_state = 0

def tps_north():        
# =============================================================================
# Bu fonksiyonda robotun kuzey noktası eğer 0 ise hedef olarak seçilir
# =============================================================================
    # List boundary control
    if(grid_search.cur_pos_x + 1 < grid_search.rows):
        # If north direction is discoverable
        if(grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x + 1
            target_pos_search.y = grid_search.cur_pos_y
            target_pos_search.direction = "north"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            cal_next_step_range.state = 0
            target_pos_search.message_send = 1
            target_pos_search.target_time = grid_search.robot.getTime()
            target_pos_search.target_state = 1
            target_pos_search.conf_check = 0
            send_sync_message()

def tps_east():
# =============================================================================
# Bu fonksiyonda robotun doğu noktası eğer 0 ise hedef olarak seçilir
# =============================================================================
    # List boundary control
    if(grid_search.cur_pos_y + 1 < grid_search.cols):
        
        if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x
            target_pos_search.y = grid_search.cur_pos_y + 1
            target_pos_search.direction = "east"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            cal_next_step_range.state = 0
            target_pos_search.message_send = 1
            target_pos_search.target_time = grid_search.robot.getTime()
            target_pos_search.target_state = 1
            target_pos_search.conf_check = 0
            send_sync_message()

def tps_south():
# =============================================================================
# Bu fonksiyonda robotun güney noktası eğer 0 ise hedef olarak seçilir
# =============================================================================
    # List boundary control
    if(grid_search.cur_pos_x - 1 >= 0):            
        if(grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x - 1
            target_pos_search.y = grid_search.cur_pos_y
            target_pos_search.direction = "south"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            cal_next_step_range.state = 0
            target_pos_search.message_send = 1
            target_pos_search.target_time = grid_search.robot.getTime()
            target_pos_search.target_state = 1
            target_pos_search.conf_check = 0
            send_sync_message()
    
def tps_west():
# =============================================================================
# Bu fonksiyonda robotun batı noktası eğer 0 ise hedef olarak seçilir
# =============================================================================
    # List boundary control
    if(grid_search.cur_pos_y - 1 >= 0):            
        if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0 and target_pos_search.found == 0):
            target_pos_search.x = grid_search.cur_pos_x
            target_pos_search.y = grid_search.cur_pos_y - 1
            target_pos_search.direction = "west"
            target_pos_search.found = 1
            target_pos_search.target_found = 1
            cal_next_step_range.state = 0
            target_pos_search.message_send = 1
            target_pos_search.target_time = grid_search.robot.getTime()
            target_pos_search.target_state = 1
            target_pos_search.conf_check = 0
            send_sync_message()

def new_coordinate():
# =============================================================================
# Bu fonksiyon bir sonraki aranacak alana geçerken doğru
# başlangıç konumlarını almasını sağlar
# =============================================================================
    
    if(search_area.target_direcition == "north"):
        search_area.new_x = 0
        search_area.new_y = target_pos_search.y
        
    if(search_area.target_direcition == "east"):
        search_area.new_x = target_pos_search.x
        search_area.new_y = 0
        
    if(search_area.target_direcition == "south"):
        search_area.new_x = grid_search.rows - 1
        search_area.new_y = target_pos_search.y
        
    if(search_area.target_direcition == "west"):
        search_area.new_x = target_pos_search.x
        search_area.new_y = grid_search.cols - 1

def extend_map():
# =============================================================================
# Bu fonksiyon grid aramasının final durumunda haritanın gidilecek
# yöne göre genişletilip bir geçiş bölgesi oluşturulmasını sağlar
# =============================================================================

    
    grid_search.extend_check = 1
    
    numberOfArea = (search_area.cur_x + 5 * search_area.cur_y)
    
    grid_search.grid_map_all[numberOfArea] = grid_search.grid_map
    
    if (search_area.target_direcition == "north"):
        grid_search.grid_map.extend(grid_search.next_grid_north)
        
    if (search_area.target_direcition == "east"):
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.grid_map.extend(grid_search.next_grid_east)
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        
    if (search_area.target_direcition == "south"):
        grid_search.next_grid_south.extend(grid_search.grid_map)
        grid_search.grid_map = grid_search.next_grid_south
        grid_search.cur_pos_x += 1
        
    if (search_area.target_direcition == "west"):
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.next_grid_west.extend(grid_search.grid_map)
        grid_search.grid_map = grid_search.next_grid_west
        grid_search.grid_map = np.transpose(grid_search.grid_map).tolist()
        grid_search.cur_pos_y += 1
        
    # draw_map(grid_search.grid_map)
    
    grid_search.next_grid_north = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_east = [[0 for i in range(grid_search.rows)]]
    grid_search.next_grid_south = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_west = [[0 for i in range(grid_search.rows)]]

def find_target_direction(): 
# =============================================================================
# Bu fonksiyon hedeflenen konumun hangi yönde olduğunu belirlememizi sağlar
# =============================================================================
    if(target_pos_search.x == grid_search.cur_pos_x + 1 and 
       target_pos_search.y == grid_search.cur_pos_y):
        target_pos_search.direction = "north"
    
    if(target_pos_search.x == grid_search.cur_pos_x and 
       target_pos_search.y == grid_search.cur_pos_y + 1):
        target_pos_search.direction = "east"
        
    if(target_pos_search.x == grid_search.cur_pos_x - 1 and 
       target_pos_search.y == grid_search.cur_pos_y):
        target_pos_search.direction = "south"
        
    if(target_pos_search.x == grid_search.cur_pos_x and 
       target_pos_search.y == grid_search.cur_pos_y - 1):
        target_pos_search.direction = "west"


def target_pos_search():
# =============================================================================
# This function find the new target position on map
# =============================================================================
    receive_message()
    
# =============================================================================
#     İlk durum olarak robotun yönünü öncelekleyip etrafındaki keşfedilebilecek
#     alan olup olmadığını kontrol eder.
# =============================================================================
    if (target_pos_search.found == 0 and target_pos_search.target_found == 0 and 
        target_pos_search.sng_state == 0 and grid_search.final_state == 0):
        
        if -0.01<grid_search.north<0.01:
            tps_north()        
            tps_east()        
            tps_west()         
            tps_south()
            
        if 1.56<grid_search.north<1.58:
            tps_east()        
            tps_north()        
            tps_south()   
            tps_west()
        
        if (-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
            tps_south()   
            tps_east()        
            tps_west()
            tps_north()
        
        if -1.58<grid_search.north<-1.56:
            tps_west()
            tps_north()        
            tps_south()   
            tps_east()
    
# =============================================================================
#     İlk durumda hedef bulunamaz ise haritadaki en yakın 0 ı bulan algoritma çalışır 
# =============================================================================
    
    if(target_pos_search.sng_state == 0 and target_pos_search.found == 0 
       and target_pos_search.target_found == 0 and grid_search.final_state == 0):
        target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                  (grid_search.cur_pos_x, grid_search.cur_pos_y), 0)
        counter = 400
        while target_pos_search.path == None and counter != 0 and target_pos_search.second_sng != 1:
            receive_message()
            # En yakın 0 konumunu bulan algoritma
            target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                      (grid_search.cur_pos_x, grid_search.cur_pos_y), 1)
            
            if target_pos_search.path != None:  target_pos_search.second_sng = 1
            counter -= 1
        
        # Eğer hedef bulunduysa
        if (target_pos_search.path != None):
            # Search near grid state
            target_pos_search.sng_state = 1
            target_pos_search.target_found = 1
            target_pos_search.sng_flag = 1 
            
            target_pos_search.path.pop(0)
            
            # Bulunan hedefin son durağını diğer robotlara gönderilmesi
            target_pos_search.sng_last_x, target_pos_search.sng_last_y = target_pos_search.path[-1]
            message = struct.pack("hhhhh",search_area.rSelf,(search_area.cur_x + 5 * search_area.cur_y),1,target_pos_search.sng_last_x,target_pos_search.sng_last_y)
            byte = 5
            byte_message = struct.pack("h",byte)
            grid_search.emitter.send(byte_message)
            grid_search.emitter.send(message)
        
        # Eğer hedef bulunamadıysa final durumuna girilirs
        else:
            grid_search.finish_time = grid_search.robot.getTime()
            print(grid_search.finish_time - grid_search.start_time)
            grid_search.final_state = 1
            
            grid_search.rot_angle = -1
            grid_search.rot_state = 1
            
            target_pos_search.second_sng = 0
            
            # Diğer robotlara final durumuna girildiği bildirilir
            byte_message = struct.pack("h",-1)
            grid_search.emitter.send(byte_message)
            
            message = struct.pack("h",search_area.rSelf)
            grid_search.emitter.send(message)
            # print("final state 1")
            # draw_map(grid_search.grid_map)
            
            grid_search.left_speed = 0
            grid_search.right_speed = 0
            
            set_speed(grid_search.left_speed,grid_search.right_speed)
            
    # sng algoritmasıyla belirlenmiş güzergahın takip edilebilmesi için her
    # hedef varımında yeni hedefin bulunur      
    if(target_pos_search.sng_state == 1):
        if(target_pos_search.sng_flag == 1):
            if(target_pos_search.path):
                target_pos_search.x, target_pos_search.y = target_pos_search.path.pop(0)
                
                target_pos_search.target_state = 1
                target_pos_search.target_time = grid_search.robot.getTime()
                target_pos_search.conf_check = 0
                
                send_sync_message()
                
                cal_next_step_range.state = 0
                target_pos_search.sng_flag = 0
                target_pos_search.message_send = 1
            # Eğer sng de belirlenen güzergah listesi bittiyse sng durumundan çıkar
            else:
                target_pos_search.sng_state = 0
                target_pos_search.target_found = 0
                target_pos_search.target_state = 0
                target_pos_search.conf = 0
                target_pos_search.conf_check = 0
                
                receive_message.r1_confirm = 0
                receive_message.r2_confirm = 0
                receive_message.r3_confirm = 0
                
                # Eğer sng sonlanmışsa haritada gidilecek nokta kalmamıştır
                # Grid search fonksiyonundaki whiledan çıkılır.
                if(grid_search.final_state == 1):
                    grid_search.exit_flag = 1                    
                    target_pos_search.message_send = 1 
                    new_coordinate()
                    
        # Belirlenen hedefin hangi yönde olduğunu belirler
        if(target_pos_search.sng_flag == 0):
            find_target_direction()
    
    # Mesaj gönderilmesi istenildiğinde harita bilgisi gönderilir
    if(target_pos_search.message_send == 1):            
        send_message()
    
    # Eğer final durumuna girildiyse
    if(grid_search.final_state == 1):
        # Diğer robotlarda final durumuna girmişlerse diğer bölgeye geçişe hazırlanılır
        if(search_area.r5_final == 1 and search_area.r6_final == 1 
           and search_area.r7_final == 1 and target_pos_search.final_wait_state == 0):
                target_pos_search.final_wait = 0
                target_pos_search.counter = 2
                target_pos_search.final_wait_state = 1
        
        if(target_pos_search.counter == 0):
            target_pos_search.final_wait = 1
        
        if target_pos_search.counter>0: target_pos_search.counter -= 1
    
    # Diğer bölgeye geçişi sağlar
    if(grid_search.final_state == 1 and target_pos_search.sng_state == 0 
       and target_pos_search.final_wait == 1):
        if(search_area.found == 0 and search_area.r5_final == 1
           and search_area.r6_final == 1 and search_area.r7_final == 1):            
            search_area.found = 1
            
            search_area.r5_final = 0
            search_area.r6_final = 0
            search_area.r7_final = 0
            
            # 5x5 lik alanda tamamlanan yer 1 işaretlenir
            search_area.map[search_area.cur_x][search_area.cur_y] = 1
            
            search_area.map_chance = 1
            
            # draw_map(search_area.map)                           
            
            # Bir sonraki gidilecek bölgenin bulunması
            cal_next_grid()
            
            # Eğer gidilecek bölge bulunamadıysa görev tamamlanır
            if(search_area.target_direcition == "empty"):
                print("mission complate")
                search_area.mission_complate = 1
                grid_search.exit_flag = 1    
                
                grid_search.grid_map_all[0]
                for i in range(25):
                    
                    temp_map = grid_search.grid_map_all[i]
                    draw_map(temp_map)
            
            # Harita gidilecek yöne doğru bir birim genişletilir
            extend_map()
            
            # Geçiş bölgesinde bulunan açıklıklar hedef olarak seçilir
            target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                      (grid_search.cur_pos_x, grid_search.cur_pos_y), 0)    
            # print(target_pos_search.path)
            if target_pos_search.path == None:
                receive_message()
                target_pos_search.path = search_near_grid(grid_search.grid_map, 
                                                      (grid_search.cur_pos_x, grid_search.cur_pos_y), 1)
            # Hedef bulunduğunda
            if (target_pos_search.path != None):
                # Search near grid state
                target_pos_search.sng_state = 1
                target_pos_search.target_found = 1
                
                # Bulunan güzergah son durağı diğer robotlara iletilir
                x, y = target_pos_search.path[-1]
                message = struct.pack("hhhhh",search_area.rSelf,(search_area.cur_x + 5 * search_area.cur_y),1,x,y)
                byte = 5
                byte_message = struct.pack("h",byte)
                grid_search.emitter.send(byte_message)
                grid_search.emitter.send(message)
                
                target_pos_search.grid_pass_last_x, target_pos_search.grid_pass_last_y = target_pos_search.path[-2]
                
                target_pos_search.path.pop(0)
                
                target_pos_search.x, target_pos_search.y = target_pos_search.path.pop(0)
                
                send_sync_message()
                
                find_target_direction()
                cal_next_step_range.state = 0
                
                target_pos_search.conf_check = 0
                target_pos_search.sng_flag = 0
                target_pos_search.message_send = 1
                
        

def search_near_grid(grid_map, start, check):
# =============================================================================
# Bu fonksiyon haritada buluna en yakın 0 ı hedef olarak belirler
# =============================================================================
    rows = len(grid_map)
    cols = len(grid_map[0])
    
    queue = collections.deque([[start]])
    seen = set([start])
    
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if grid_map[x][y] == grid_search.goal:
            return path
        
        # Burada yalnızca 1 ve kendi üzerinden güzergah belirleyebilir
        if check == 0:
            for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if (0 <= x2 < rows and 0 <= y2 < cols and 
                    (grid_map[x2][y2] == search_area.rSelf or grid_map[x2][y2] == 1 or grid_map[x2][y2] == 0)
                    and (x2, y2) not in seen):
                    queue.append(path + [(x2, y2)])
                    seen.add((x2, y2))
        
        # Burada duvar olmayan her yerden güzergah belirler
        else:                      
            for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if 0 <= x2 < rows and 0 <= y2 < cols and grid_map[x2][y2] != grid_search.wall and (x2, y2) not in seen:
                    queue.append(path + [(x2, y2)])
                    seen.add((x2, y2))
                    
def cal_next_step_range():
# =============================================================================
# This function calculate next step range value or 
# decides turn direction to target position direction
# =============================================================================
    receive_message()
    
    # If target position found and range not calculated before
    if((target_pos_search.found == 1 or target_pos_search.sng_state == 1) and cal_next_step_range.state == 0 and 
       target_pos_search.conf == 1):
        
        # Hedef olarak belirlenen nokta engel tespit edilirse yeni hedef bul
        if(grid_search.grid_map[target_pos_search.x][target_pos_search.y] == 3):
            
            target_pos_search.sng_state = 0
            target_pos_search.target_found = 0
            target_pos_search.found = 0
            target_pos_search.direction = "empty"
            target_pos_search.target_state = 0
            target_pos_search.conf = 0
            target_pos_search.conf_check = 0
            
            cal_next_step_range.state = 1
            
            grid_search.rot_angle = -1
            grid_search.rot_state = 1
            grid_search.left_speed = 0
            grid_search.right_speed = 0
            
            receive_message.r1_confirm = 0
            receive_message.r2_confirm = 0
            receive_message.r3_confirm = 0
            
            set_speed(grid_search.left_speed,grid_search.right_speed)
            
            # print("sng reset")
        
        # Hedefe hareketi sağlar
        elif cal_next_step_range.state == 0:            
            # If target direction same to robot directon 
            if((target_pos_search.direction == "north" and -0.01<grid_search.north<0.01)
                or (target_pos_search.direction == "east" and 1.56<grid_search.north<1.58)
                or (target_pos_search.direction == "south" and (-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15))
                or (target_pos_search.direction == "west" and -1.58<grid_search.north<-1.56)):
                    # Left or right wheel range is current positional sensor value + increment value
                    
                    cal_next_step_range.rangeLeft = grid_search.ps_left_value + grid_search.increment_range
                    cal_next_step_range.rangeRight = grid_search.ps_right_value + grid_search.increment_range
                    # print('cal range for next step')
                    cal_next_step_range.state = 1
                    grid_search.movement_check = 1
                    
                    grid_search.rot_state = 0
                    
                    
            # Not same and if target direction is north then turn north
            elif(target_pos_search.direction == "north"):
                grid_search.rot_angle = 0
                grid_search.rot_state = 1
                cal_next_step_range.state = 1
                grid_search.movement_check = 0
                
            # Not same and if target direction is north then turn east
            elif(target_pos_search.direction == "east"):
                grid_search.rot_angle = 1
                grid_search.rot_state = 1
                cal_next_step_range.state = 1
                grid_search.movement_check = 0
            
            # Not same and if target direction is north then turn south
            elif(target_pos_search.direction == "south"):
                grid_search.rot_angle = 2
                grid_search.rot_state = 1
                cal_next_step_range.state = 1
                grid_search.movement_check = 0
            
            # Not same and if target direction is north then turn west
            elif(target_pos_search.direction == "west"):
                grid_search.rot_angle = 3
                grid_search.rot_state = 1
                cal_next_step_range.state = 1
                grid_search.movement_check = 0
            # Eğer robot hedef yöne boş olursa dönme engellenir bekleme duruma geçilmiştir
            elif(target_pos_search.direction == "empty"):
                grid_search.rot_angle = -1
                grid_search.movement_check = 0
    
    # Eğer diğer robotlardan belirlenen hedef için onay gelmezse bekleme durumuna geçer
    elif(target_pos_search.conf == 0):       
        if target_pos_search.conf_check == 0:
            if target_pos_search.conf_wait_cont == -1:   target_pos_search.conf_wait_cont = 400
            
            # Bekleme
            if target_pos_search.conf_wait_cont > 0:    
                target_pos_search.conf_wait_cont -= 1                    
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                grid_search.movement_check = 0
                grid_search.rot_angle = -1
                grid_search.rot_state = 1
            
            # Beklerken her 25 zaman adımında bir yeniden istek gönderilir
            if target_pos_search.conf_wait_cont % 25 == 0:
                send_sync_message()
                if target_pos_search.conf == 0:
                    receive_message.r1_confirm = 0
                    receive_message.r2_confirm = 0
                    receive_message.r3_confirm = 0
            
            # Eğer bekleme zaman aşımına uğrarsa yeni hedef ya da güzergah bulunur
            if target_pos_search.conf_wait_cont == 0:   
                
                target_pos_search.conf_check = 1
                target_pos_search.sng_state = 0
                target_pos_search.target_found = 0
                target_pos_search.target_state = 0
                target_pos_search.conf_wait_cont = -1
                
                receive_message.r1_confirm = 0
                receive_message.r2_confirm = 0
                receive_message.r3_confirm = 0

                cal_next_step_range.state = 0
                
                target_pos_search.direction = "empty"
                # print("confirm wait end")
        
        set_speed(grid_search.left_speed,grid_search.right_speed)
        
def rotation_direction(angle):
# =============================================================================
# This function detect rotate direction according to the direction of the return
# =============================================================================
    
    # Direction of return is north
    if(angle == 0):
        if((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
        elif(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
    
    # Direction of return is east
    if(angle == 1):
        if(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
        elif((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
    
    # Direction of return is south     
    if(angle == 2):
        
        if(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.slow = 45
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.check = 1
            
        elif(-1.58<grid_search.north<-1.56 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.slow = 45
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.check = 1
    
    # Direction of return is west
    if(angle == 3):
        
        if(1.56<grid_search.north<1.58 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = 2 * grid_search.const_counter - 1
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 91
            rotation_direction.check = 1
            
        elif(-0.01<grid_search.north<0.01 and rotation_direction.check == 0):
            grid_search.right_speed = grid_search.max_speed/5
            grid_search.left_speed = -grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
            
        elif((-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15) and rotation_direction.check == 0):
            grid_search.right_speed = -grid_search.max_speed/5
            grid_search.left_speed = grid_search.max_speed/5
            rotation_direction.counter = grid_search.const_counter
            rotation_direction.counter_start = rotation_direction.counter
            rotation_direction.slow = 45
            rotation_direction.check = 1
    
    # Daha hassas bir dönüş için dönülecek yöne yakınlaşınca yavaşlar
    if(rotation_direction.counter < rotation_direction.counter_start - rotation_direction.slow and 
       rotation_direction.slow_check == 0):
        grid_search.left_speed /= 48
        grid_search.right_speed /= 48       
        rotation_direction.slow_check = 1
        send_message()
        receive_message()
    # Tam 90 ve 180 derece dönüşe yakın değerler için ikinci yavaşlatma
    if rotation_direction.counter == 0 and rotation_direction.slow_check2 == 0:
        grid_search.left_speed /= 10
        grid_search.right_speed /= 10
        rotation_direction.slow_check2 = 1
        # send_message()
        receive_message()

def check_front_ds_val():
    # Mesafe sensörlerinin çok yakın robotları tespit etmesi
    for i in range(14):
        if grid_search.ds_front_values[i] < 280:
            return 1
    
    return 0

def robot_direction():
    # robotun hangi yöne baktığını tespit eder
    if -0.01<grid_search.north<0.01:    return "north"
    if 1.56<grid_search.north<1.58:    return "east"
    if (-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):    return "south"
    if -1.58<grid_search.north<-1.56:    return "west"

def grid_pass_check():
# =============================================================================
# Bu fonksiyon grid geçişlerinde geçiş bölgesinin hedef bölge tarafında robot 
# olup olmadığının kontrolünü sağlar
# =============================================================================
    
    if(search_area.target_direcition == "north"):   
        targetX = 0
        targetY = target_pos_search.y
        
    if(search_area.target_direcition == "east"):    
        targetX = target_pos_search.x
        targetY = 0
        
    if(search_area.target_direcition == "south"):   
        targetX = grid_search.rows - 1
        targetY = target_pos_search.y
        
    if(search_area.target_direcition == "west"):   
        targetX = target_pos_search.x
        targetY = grid_search.cols - 1
    
    areaNumber = search_area.target_x + 5 * search_area.target_y
    
    if(grid_search.grid_map_all[areaNumber][targetX][targetY] == search_area.r1 or
       grid_search.grid_map_all[areaNumber][targetX][targetY] == search_area.r2 or
       grid_search.grid_map_all[areaNumber][targetX][targetY] == search_area.r3):
        return 1
    else:
        return 0

def rotate_check():
# =============================================================================
# This function controls the rotational status and 
# decides when it will stop according to the rotation angle and 
# if the robot is not in the case of rotation, it should go forward.
# =============================================================================
    
    # If robot come to rotate state
    if(grid_search.rot_state == 1):
                       
        # Define left and right range to infinite
        cal_next_step_range.rangeLeft = float('inf')
        cal_next_step_range.rangeRight = float('inf')
        
        # If rotate angle is 0
        if(grid_search.rot_angle == 0):
            
            # Rotation direction according to target rotation direction
            rotation_direction(0) 
                           
            # Robot then turn north
            if(-0.00009<grid_search.north<0.0009):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                block_check()
                
                
        
        # If rotate angle is 1
        if(grid_search.rot_angle == 1):
            
            # Rotation direction according to target rotation direction
            rotation_direction(1)
            
            # Robot then turn east
            if(1.57075<grid_search.north<1.57085):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                block_check()
        
        # If rotate angle is 2
        if(grid_search.rot_angle == 2):
            
            # Rotation direction according to target rotation direction
            rotation_direction(2)
            
            # Robot then turn south
            if((-3.14168<grid_search.north<-3.14152 or 3.14152<grid_search.north<3.14161)):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                block_check()
                
        
        # If rotate angle is 3
        if(grid_search.rot_angle == 3):
            
            # Rotation direction according to target rotation direction
            rotation_direction(3)
            
            # Robot then turn west
            if(-1.57085<grid_search.north<-1.57075):
                grid_search.rot_state = 0
                grid_search.left_speed = 0
                grid_search.right_speed = 0
                cal_next_step_range.state = 0
                rotation_direction.check = 0
                rotation_direction.speed_set = 0
                rotation_direction.counter = -1
                rotation_direction.slow_check = 0
                rotation_direction.slow_check2 = 0
                block_check()
                
    # Else go forward
    else:
        
        # Geçiş bölgesinde diğer bölgede robot varsa beklemeye geçer
        if(grid_search.final_state == 1 and target_pos_search.grid_pass_last_x == grid_search.cur_pos_x and 
        target_pos_search.grid_pass_last_y == grid_search.cur_pos_y and rotate_check.final_check == 0):
            if(grid_pass_check()):
                if rotate_check.counter == -1:   rotate_check.counter = 500
                if rotate_check.counter > 0:    
                    rotate_check.counter -= 1                    
                    grid_search.left_speed = 0
                    grid_search.right_speed = 0
                    grid_search.forward_check = 0
                
                if rotate_check.counter == 0:   
                    rotate_check.final_check = 1
                    grid_search.forward_check = 1
                
            else:
                grid_search.forward_check = 1
        
        # Hedef olarak belirlenen nokta bir başka robot tarafından işaretlenmiş ise bekle
        if(grid_search.grid_map[target_pos_search.x][target_pos_search.y] == search_area.r1
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == search_area.r2
           or grid_search.grid_map[target_pos_search.x][target_pos_search.y] == search_area.r3):                 
            grid_search.left_speed = 0
            grid_search.right_speed = 0
            
            if rotate_check.counter == -1:   rotate_check.counter = 400
            if rotate_check.counter > 0:    rotate_check.counter -= 1
            # Zaman aşımında yeni hedef veya güzergah tespit et
            if (rotate_check.counter == 0):
                
                target_pos_search.sng_state = 0
                target_pos_search.target_found = 0
                target_pos_search.target_state = 0
                target_pos_search.conf = 0
                target_pos_search.direction = "empty"

                cal_next_step_range.state = 0
                
                receive_message.r1_confirm = 0
                receive_message.r2_confirm = 0
                receive_message.r3_confirm = 0
                
                rotate_check.counter = -1
                
                if(grid_search.grid_map[target_pos_search.sng_last_x][target_pos_search.sng_last_y] == 1):
                    grid_search.grid_map[target_pos_search.sng_last_x][target_pos_search.sng_last_y] = 0
                
                # print("It's tooooo much time")
        
        # Eğer mesafe sensörü robot algılamış ise beklemeye geç
        elif check_front_ds_val() and rotate_check.ds_check == 0:
            
            if rotate_check.counter == -1:   rotate_check.counter = 500
            if rotate_check.counter > 0:    
                rotate_check.counter -= 1                    
                grid_search.left_speed = 0
                grid_search.right_speed = 0
            
            # Zaman aşımı harekete devam et
            if rotate_check.counter == 0:   
                rotate_check.ds_check = 1
        
        
        else: 
            if(target_pos_search.x == grid_search.cur_pos_x 
               and target_pos_search.y == grid_search.cur_pos_y):
                pass
            elif(grid_search.forward_check == 1):
                grid_search.left_speed = grid_search.max_speed
                grid_search.right_speed = grid_search.max_speed
                

def tumble_check():
# =============================================================================
# This function check the robot tumble status and 
# decides when reverse the wheel speed direction to the robot inertial unit 
# sensor values
# =============================================================================
    if((grid_search.imu_value[0]<-3 or grid_search.imu_value[0]>3) and grid_search.tumble_state == 0):
        grid_search.left_speed *= -1
        grid_search.right_speed *= -1
        grid_search.tumble_state = 1
    if(not(grid_search.imu_value[0]<-3 or grid_search.imu_value[0]>3) and grid_search.tumble_state == 1):
        grid_search.left_speed *= -1
        grid_search.right_speed *= -1
        grid_search.tumble_state = 0

def rotate_pos_check():
# =============================================================================
# This function check robot come to target position and reset some values and
# check any block in around the robot
# =============================================================================
    # If robot go to the target position 
    if(grid_search.ps_left_value > cal_next_step_range.rangeLeft-0.002 and grid_search.movement_check == 1):
        # Reset variables
        grid_search.left_speed = 0
        grid_search.right_speed = 0 
        grid_search.movement_check = 0
        
        target_pos_search.found = 0
        target_pos_search.target_state = 0
        target_pos_search.conf = 0
        target_pos_search.conf_check = 0
        target_pos_search.direction = ""
        
        cal_next_step_range.state = 0
        
        rotate_pos_check.message_check = 0
        rotate_pos_check.message_check2 = 0
        rotate_pos_check.message_check3 = 0
        
        rotate_check.ds_check = 0
        rotate_check.final_check = 0
        rotate_check.counter = -1
        target_pos_search.conf_wait_cont = -1
        
        receive_message.r1_confirm = 0
        receive_message.r2_confirm = 0
        receive_message.r3_confirm = 0
        
        # print("reset")
        if(target_pos_search.sng_state == 1):
            if(target_pos_search.sng_flag == 0):
                target_pos_search.sng_flag = 1
        else:
            target_pos_search.target_found = 0
        
        grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y] = 1
        
        # Now robot current position is target postion
        grid_search.cur_pos_x = target_pos_search.x
        grid_search.cur_pos_y = target_pos_search.y
        
        send_message()
        
        # Check front, left or right sight of robot for any block
        block_check()
    
    if grid_search.movement_check == 1:
        rangeVal = cal_next_step_range.rangeLeft/4
        if (grid_search.ps_left_value > rangeVal and rotate_pos_check.message_check == 0):
            send_message()
            rotate_pos_check.message_check = 1
        
        if (grid_search.ps_left_value > rangeVal * 2 and rotate_pos_check.message_check2 == 0):
            send_message()
            rotate_pos_check.message_check2 = 1
            
        if (grid_search.ps_left_value > rangeVal * 3 and rotate_pos_check.message_check3 == 0):
            send_message()
            rotate_pos_check.message_check3 = 1
        
def block_check():
# =============================================================================
# Bu fonksiyon robotun sensörlerinde engel tespit edip etmediği belirler ve 
# bunu haritada güncelleyip diğer robotlara bildirir
# =============================================================================
    if grid_search.final_state == 0:
        receive_message()
        # Front Block detection
        for i in range(14):
            if(grid_search.ds_front_values[i] < 1453):
                # If direction of robot is north
                if(-0.01<grid_search.north<0.01):
                    if(grid_search.cur_pos_x + 1 < grid_search.rows):
                        if (grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0):
                            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3 
                        
                        if(grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] not in (search_area.r1, search_area.r2, search_area.r3, grid_search.wall)):
                            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3
                            
                            target_pos_search.sng_state = 0
                            target_pos_search.target_found = 0
                            target_pos_search.found = 0
                            target_pos_search.direction = "empty"
                            target_pos_search.target_state = 0
                            target_pos_search.conf = 0
                            target_pos_search.conf_check = 0
                            
                            cal_next_step_range.state = 0
                            
                            grid_search.rot_angle = -1
                            grid_search.rot_state = 1
                            grid_search.left_speed = 0
                            grid_search.right_speed = 0
                            
                            receive_message.r1_confirm = 0
                            receive_message.r2_confirm = 0
                            receive_message.r3_confirm = 0
                            
                            set_speed(grid_search.left_speed,grid_search.right_speed)
                            
                    elif(grid_search.next_grid_north[0][grid_search.cur_pos_y] == 0):
                        
                        grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                        
                        byte_message = struct.pack("h",-2)
                        grid_search.emitter.send(byte_message)
                        message = struct.pack("hh",1,grid_search.cur_pos_y)
                        grid_search.emitter.send(message)
                            
                # If direction of robot is east
                if(1.56<grid_search.north<1.58):
                    if(grid_search.cur_pos_y + 1 < grid_search.cols):
                        if (grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0):
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
                        
                        if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] not in (search_area.r1, search_area.r2, search_area.r3, grid_search.wall)):
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
                            
                            target_pos_search.sng_state = 0
                            target_pos_search.target_found = 0
                            target_pos_search.found = 0
                            target_pos_search.direction = "empty"
                            target_pos_search.target_state = 0
                            target_pos_search.conf = 0
                            target_pos_search.conf_check = 0
                            
                            cal_next_step_range.state = 0
                            
                            receive_message.r1_confirm = 0
                            receive_message.r2_confirm = 0
                            receive_message.r3_confirm = 0
                            
                            grid_search.rot_angle = -1
                            grid_search.rot_state = 1
                            grid_search.left_speed = 0
                            grid_search.right_speed = 0
                            
                            set_speed(grid_search.left_speed,grid_search.right_speed)
                            
                    elif (grid_search.next_grid_east[0][grid_search.cur_pos_x] == 0):
                        
                        grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                        
                        byte_message = struct.pack("h",-2)
                        grid_search.emitter.send(byte_message)
                        message = struct.pack("hh",2,grid_search.cur_pos_x)
                        grid_search.emitter.send(message)
                    
                # If direction of robot is south
                if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
                    if(grid_search.cur_pos_x - 1 >= 0):
                        if (grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0):
                            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
                            
                        if(grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] not in (search_area.r1, search_area.r2, search_area.r3, grid_search.wall)):
                            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
                            
                            target_pos_search.sng_state = 0
                            target_pos_search.target_found = 0
                            target_pos_search.found = 0
                            target_pos_search.direction = "empty"
                            target_pos_search.target_state = 0
                            target_pos_search.conf = 0
                            target_pos_search.conf_check = 0
                            
                            cal_next_step_range.state = 0
                            
                            receive_message.r1_confirm = 0
                            receive_message.r2_confirm = 0
                            receive_message.r3_confirm = 0
                            
                            grid_search.rot_angle = -1
                            grid_search.rot_state = 1
                            grid_search.left_speed = 0
                            grid_search.right_speed = 0
                            
                            set_speed(grid_search.left_speed,grid_search.right_speed)
                            
                    elif (grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0):
                        grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                        
                        byte_message = struct.pack("h",-2)
                        grid_search.emitter.send(byte_message)
                        message = struct.pack("hh",3,grid_search.cur_pos_y)
                        grid_search.emitter.send(message)
                    
                # If direction of robot is west
                if(-1.58<grid_search.north<-1.56):
                    if(grid_search.cur_pos_y - 1 >= 0):
                        if (grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0):
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
                            
                        if(grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] not in (search_area.r1, search_area.r2, search_area.r3, grid_search.wall)):
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
                            
                            target_pos_search.sng_state = 0
                            target_pos_search.target_found = 0
                            target_pos_search.found = 0
                            target_pos_search.direction = "empty"
                            target_pos_search.target_state = 0
                            target_pos_search.conf = 0
                            target_pos_search.conf_check = 0
                            
                            cal_next_step_range.state = 0
                            
                            receive_message.r1_confirm = 0
                            receive_message.r2_confirm = 0
                            receive_message.r3_confirm = 0
                            
                            grid_search.rot_angle = -1
                            grid_search.rot_state = 1
                            grid_search.left_speed = 0
                            grid_search.right_speed = 0
                            
                            set_speed(grid_search.left_speed,grid_search.right_speed)
                            
                    elif (grid_search.next_grid_west[0][grid_search.cur_pos_x] == 0):
                        grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                        
                        byte_message = struct.pack("h",-2)
                        grid_search.emitter.send(byte_message)
                        message = struct.pack("hh",0,grid_search.cur_pos_x)
                        grid_search.emitter.send(message)
                    
        # Right Block detection
        for i in range(5):     
            if(grid_search.ds_right_values[i] < 1675):
            
                # If direction of robot is north
                if(-0.01<grid_search.north<0.01):
                    if(grid_search.cur_pos_y + 1 < grid_search.rows):
                        if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
                    
                    elif grid_search.cur_pos_x < grid_search.rows:
                        if grid_search.next_grid_east[0][grid_search.cur_pos_x] == 0:                
                            grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",2,grid_search.cur_pos_x)
                            grid_search.emitter.send(message)
                
                # If direction of robot is east
                if(1.56<grid_search.north<1.58):
                    if(grid_search.cur_pos_x - 1 >= 0):
                        if grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
                    
                    elif grid_search.cur_pos_y < grid_search.rows:
                        if grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                            grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",3,grid_search.cur_pos_y)
                            grid_search.emitter.send(message)
                
                # If direction of robot is south
                if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15):
                    if(grid_search.cur_pos_y - 1 >= 0):
                        if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
                    
                    elif grid_search.cur_pos_x < grid_search.rows:
                        if grid_search.next_grid_west[0][grid_search.cur_pos_x] == 0:
                            grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",0,grid_search.cur_pos_x)
                            grid_search.emitter.send(message)
                
                # If direction of robot is west
                if(-1.58<grid_search.north<-1.56): 
                    if(grid_search.cur_pos_x + 1 < grid_search.rows):
                        if grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3     
                        
                    elif grid_search.cur_pos_y < grid_search.rows:
                        if grid_search.next_grid_north[0][grid_search.cur_pos_y] == 0:
                            grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",1,grid_search.cur_pos_y)
                            grid_search.emitter.send(message)
                    
        
        # Left Block detection
        for i in range(5):
            if(grid_search.ds_left_values[i] < 1675):
            
                # If direction of robot is north
                if(-0.01<grid_search.north<0.01):
                    if(grid_search.cur_pos_y - 1 >= 0):
                        if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y - 1] = 3
                    
                    elif grid_search.cur_pos_x < grid_search.rows:
                        if grid_search.next_grid_west[0][grid_search.cur_pos_x] == 0:
                            grid_search.next_grid_west[0][grid_search.cur_pos_x] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",0,grid_search.cur_pos_x)
                            grid_search.emitter.send(message)
                
                # If direction of robot is east
                if(1.56<grid_search.north<1.58):
                    if(grid_search.cur_pos_x + 1 < grid_search.rows):
                        if grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x + 1][grid_search.cur_pos_y] = 3     
                        
                    elif grid_search.cur_pos_y < grid_search.rows:
                        if grid_search.next_grid_north[0][grid_search.cur_pos_y] == 0:
                            grid_search.next_grid_north[0][grid_search.cur_pos_y] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",1,grid_search.cur_pos_y)
                            grid_search.emitter.send(message)
                
                # If direction of robot is south
                if(-3.15<grid_search.north<-3.13 or 3.13<grid_search.north<3.15): 
                    if(grid_search.cur_pos_y + 1 < grid_search.rows):
                        if grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x][grid_search.cur_pos_y + 1] = 3
                    
                    elif grid_search.cur_pos_x < grid_search.rows:
                        if grid_search.next_grid_east[0][grid_search.cur_pos_x] == 0:
                            grid_search.next_grid_east[0][grid_search.cur_pos_x] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",2,grid_search.cur_pos_x)
                            grid_search.emitter.send(message)
                
                # If direction of robot is west
                if(-1.58<grid_search.north<-1.56):
                    if(grid_search.cur_pos_x - 1 >= 0):
                        if grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] == 0:
                            grid_search.grid_map[grid_search.cur_pos_x - 1][grid_search.cur_pos_y] = 3
                    
                    elif grid_search.cur_pos_y < grid_search.rows:
                        if grid_search.next_grid_south[0][grid_search.cur_pos_y] == 0:
                            grid_search.next_grid_south[0][grid_search.cur_pos_y] = 3
                            
                            byte_message = struct.pack("h",-2)
                            grid_search.emitter.send(byte_message)
                            message = struct.pack("hh",3,grid_search.cur_pos_y)
                            grid_search.emitter.send(message)

def cal_next_grid():
# =============================================================================
# Bu fonksiyon bir sonraki taranacak alanın belirlenmesini sağlar ve hedef 
# taranacak bölgenin yönünü tayin eder
# =============================================================================
    if(cal_next_grid.found == 0):
        if(0 <= search_area.cur_x + 1 < search_area.rows):
            if(search_area.map[search_area.cur_x + 1][search_area.cur_y] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x + 1
                search_area.target_y = search_area.cur_y
                search_area.target_direcition = "north"
                cal_next_grid.found = 1
            
        if(0 <= search_area.cur_y + 1 < search_area.cols):  
            if(search_area.map[search_area.cur_x][search_area.cur_y + 1] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x
                search_area.target_y = search_area.cur_y + 1
                search_area.target_direcition = "east"
                cal_next_grid.found = 1
        
        if(0 <= search_area.cur_x - 1):
            if(search_area.map[search_area.cur_x - 1][search_area.cur_y] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x - 1
                search_area.target_y = search_area.cur_y
                search_area.target_direcition = "south"
                cal_next_grid.found = 1
        
        if(0 <= search_area.cur_y - 1):
            if(search_area.map[search_area.cur_x][search_area.cur_y - 1] == 0 and 
               cal_next_grid.found == 0):
                search_area.target_x = search_area.cur_x
                search_area.target_y = search_area.cur_y - 1
                search_area.target_direcition = "west"
                cal_next_grid.found = 1 
                
        # print("exit::",cal_next_grid.found)


def search_area(robot):
    search_area.mission_complate = 0
    
    search_area.cols = 5
    search_area.rows = 5
    
    # 5x5 lik harita
    search_area.map = [[0 for i in range(search_area.cols)] for j in range(search_area.rows)]
    
    search_area.map_chance = 0
    
    search_area.cur_x = 0
    search_area.cur_y = 0
    
    search_area.target_x = 0
    search_area.target_y = 0
    
    search_area.new_x = 0
    search_area.new_y = 3
    
    search_area.rSelf = 7
    search_area.r1 = 6
    search_area.r2 = 5
    search_area.r3 = 4 
    
       
    
    search_area.start_time = robot.getTime()
    
    while search_area.mission_complate != 1:
        search_area.target_direcition = "empty"
                
        cal_next_grid.found = 0
        
        # Her bir taranacak alan için çağırılan fonksiyon
        grid_search(robot, search_area.new_x, search_area.new_y)
        
        
        # Belirlene yöne göre robot hedef konumlarının optimizasyonu
        if(search_area.target_direcition == "north"):   target_pos_search.x = 0
        
        if(search_area.target_direcition == "east"):    target_pos_search.y = 0
            
        if(search_area.target_direcition == "south"):   target_pos_search.x = grid_search.rows - 1
            
        if(search_area.target_direcition == "west"):   target_pos_search.y = grid_search.cols - 1
        
        # 5x5 lik harita mevcut konumun güncellenmesi
        search_area.cur_x = search_area.target_x
        search_area.cur_y = search_area.target_y
        
        search_area.map_chance = 0
        
    search_area.finish_time = robot.getTime()    
    
    print(search_area.finish_time - search_area.start_time)
    
        

if __name__ == "__main__":
    # Call robot and define robot variable    
    my_robot = Supervisor()    
    
    # Simülasyon robot ve kazazede düğümleri
    robot_node = my_robot.getFromDef("MY_ROBOT_7")
    survivor_node = my_robot.getFromDef("survivor")
    survivor2_node = my_robot.getFromDef("survivor(1)")
    survivor3_node = my_robot.getFromDef("survivor(2)")
    
    # Düğümlerin bulunmama kontrolü
    if survivor_node is None:
        sys.stderr.write("No DEF survivor node found in the current world file\n")
        sys.exit(1)
        
    if survivor2_node is None:
        sys.stderr.write("No DEF survivor(1) node found in the current world file\n")
        sys.exit(2)
        
    if survivor3_node is None:
        sys.stderr.write("No DEF survivor(2) node found in the current world file\n")
        sys.exit(3)
    
    # Düğümlerin konum bilgisi alanın çağırılması
    robot_trans_field = robot_node.getField("translation")
    s_trans_field = survivor_node.getField("translation")
    s2_trans_field = survivor2_node.getField("translation")
    s3_trans_field = survivor3_node.getField("translation")
    
    # Kazazede konumları
    search_area.s_pos = s_trans_field.getSFVec3f()
    search_area.s2_pos = s2_trans_field.getSFVec3f()
    search_area.s3_pos = s3_trans_field.getSFVec3f()
    
    search_area.s_found = 0
    search_area.s2_found = 0
    search_area.s3_found = 0
    
    grid_search.const_counter = 65
    grid_search.forward_check = 1
    cal_next_step_range.rangeLeft = 0
    cal_next_step_range.rangeRight = 0
    
    cal_next_step_range.state = 0
    cal_next_step_range.range_fix = 0
    
    target_pos_search.found = 0
    target_pos_search.x = 0
    target_pos_search.y = 0
    target_pos_search.sng_last_x = 0
    target_pos_search.sng_last_y = 0
    target_pos_search.sng_state = 0
    target_pos_search.second_sng = 0
    target_pos_search.target_found = 0
    target_pos_search.sng_flag = 0
    target_pos_search.path = [] 
    target_pos_search.counter = 0    
    target_pos_search.target_time = 0
    target_pos_search.target_state = 0
    target_pos_search.conf = 0
    target_pos_search.conf_wait_cont = -1
    target_pos_search.conf_check = 0
    target_pos_search.message_send = 0
    target_pos_search.grid_pass_last_x = 0
    target_pos_search.grid_pass_last_y = 0
    
    cal_next_grid.found = 0
        
    search_area.found = 0
    search_area.r5_final = 0
    search_area.r6_final = 0
    search_area.r7_final = 0
    
    rotate_check.counter = -1
    rotate_check.ds_check = 0
    rotate_check.final_check = 0
    
    rotate_pos_check.message_check = 0
    rotate_pos_check.message_check2 = 0
    rotate_pos_check.message_check3 = 0
    
    rotation_direction.check = 0
    rotation_direction.speed_set = 0
    rotation_direction.counter = -1
    rotation_direction.slow = 0
    rotation_direction.slow_check = 0
    rotation_direction.slow_check2 = 0
    rotation_direction.counter_start = 0
    
    receive_message.r1_confirm = 0
    receive_message.r2_confirm = 0
    receive_message.r3_confirm = 0
    
    grid_search.rows = 8
    grid_search.cols = 8
    
    # Tüm 8x8 haritaların tutulduğu 3 boyutlu liste
    grid_search.grid_map_all = [[[0 for col in range(8)] for col in range(8)] for row in range(25)]      
    
    # Grid geçiş bölgeleri için tutulan listeler
    grid_search.next_grid_north = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_east = [[0 for i in range(grid_search.rows)]]
    grid_search.next_grid_south = [[0 for i in range(grid_search.cols)]]
    grid_search.next_grid_west = [[0 for i in range(grid_search.rows)]]
    
    # Start grid search proccess
    search_area(my_robot)
    
    