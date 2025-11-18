import rclpy
import DR_init
import copy
from math import sin, cos, pi

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"  # 본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  # 본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import (
        ON,
        OFF,
        DR_MV_MOD_REL,
        set_digital_output,
        wait,
        posj,
        posx,
        movej,
        movel,
    )
    from dsr_gss.firebase_utils import (
        debug,
        info,
        warning,
        error,
        increase_planted_count,
        exit,
    )

    # === Method 정의 부분 ==================================================
    def move_to_home():
        movej(posj(0, 0, 90, 0, 90, 0), acc=ACC, vel=VELOCITY)

    def open_gripper():
        # 컨트롤러 DO + 툴 DO
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.4)

    def close_gripper():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.4)

    # === Step scenario 정의 부분 ===========================================
    def step1():
        # TODO: 강종아
        pass

    def step2():
        # ===== 정의 =====
        safe_dz = 125.0  # 위로 들어올리는 높이
        rx = 180.0
        ry = -150.0
        rz = 90.0

        vel_approach = 100.0
        acc_approach = 100.0
        vel_work = 30.0
        acc_work = 60.0

        pz = 250.0  # z_pick
        tz = 250.0  # z_place
        space_gap = 100

        pick_points = [
            # (255.0, -202.0),  # 1번
            (370.0, -202.0),  # 3번
            (370.0, -72.0),  # 2번
        ]

        place_points = [
            # (556.24, 161.58),  # 1번
            (556.24, -138.42),  # 3번
            (256.24, 161.58),  # 2번
        ]

        if len(pick_points) != len(place_points):
            error("Pick/Place 포인트의 갯수가 일치하지 않습니다. 작업을 건너뜁니다.")
            return

        n = len(pick_points)

        # ===== 동작 =====
        info("===== 묘목 심기 시작 =====")
        move_to_home()
        for i in range(n):
            px, py = pick_points[i]
            tx, ty = place_points[i]
            info(f"[{i+1}/{n}] pick ({px}, {py}, {pz}) -> place ({tx}, {ty}, {tz})")

            first_pick_up = posx(px - space_gap, py, pz + safe_dz, rx, ry, rz)
            first_place_up = posx(tx, ty, tz + safe_dz, rx, ry, rz)

            # ---- 접근 & 집기 ----
            open_gripper()
            movel(first_pick_up, vel=vel_approach, acc=acc_approach)
            movel(
                posx(space_gap, 0, -120, 0, 0, 0),
                mod=DR_MV_MOD_REL,
                vel=vel_work,
                acc=acc_work,
            )
            close_gripper()
            movel(
                posx(0, 0, 200, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )

            # ---- 이동 & 놓기 ----
            movel(first_place_up, vel=vel_approach, acc=acc_approach)
            movel(
                posx(0, 0, -100, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )
            open_gripper()
            movel(
                posx(0, 0, 200, 0, 0, 0), mod=DR_MV_MOD_REL, vel=vel_work, acc=acc_work
            )
            movel(
                posx(-space_gap, 0, 0, 0, 0, 0),
                mod=DR_MV_MOD_REL,
                vel=vel_work,
                acc=acc_work,
            )

            # 심은 묘목 갯수 증가
            increase_planted_count()

        move_to_home()
        info("===== 묘목 심기 종료 =====")

    def step3():
        info("===== 땅 다지기 시작 =====")
        from DSR_ROBOT2 import (
            movej,
            movel,
            posj,
            get_current_posx,
            movec,
            posx,
            set_digital_output,
            wait,
            ON,
            OFF,
            set_ref_coord,
            DR_FC_MOD_REL,
            DR_MV_MOD_ABS,
            DR_BASE,
        )

        # # home 위치로 돌아가는 작업
        # # 경우에 따라 삭제 가능
        j0 = posj(0, 0, 90, 0, 90, 0)
        movej(j0, vel=60, acc=30)
        wait(1)
        movel([0, 0, 200, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
        wait(1)

        movej([-180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)

        ## gripping 동작 추가
        tool_pos = posx(-462.19, 9.11, 478.33, 0.0, 180.0, 0.0)  # 잡을 위치
        tool_up = copy.deepcopy(tool_pos)
        tool_up[2] += 150.0
        pc0, _ = get_current_posx()
        # movel(tool_up, vel=60, acc=100, mod=DR_MV_MOD_ABS)
        # 쪼개기
        movel(
            [tool_up[0], tool_up[1], pc0[2], pc0[3], pc0[4], pc0[5]],
            vel=60,
            acc=100,
            mod=DR_MV_MOD_ABS,
        )
        movel(
            [tool_up[0], tool_up[1], min(tool_up[2], pc0[2]), pc0[3], pc0[4], pc0[5]],
            vel=60,
            acc=100,
            mod=DR_MV_MOD_ABS,
        )

        # gripper 열기
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(3)

        movel(tool_pos, vel=60, acc=100, mod=DR_MV_MOD_ABS)
        # wait(2)
        # gripper 닫기
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(3)
        movel([0, 0, 150, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
        movel([100, 0, 0, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
        # movel(tool_up, vel=60, acc=100)
        wait(1)

        movej([180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)
        ##
        # 나무 위치들
        treeLocations = [
            posx(356.24, 161.58, 340.0, 0.0, 180.0, 0.0),
            posx(656.24, 161.58, 340.0, 0.0, 180.0, 0.0),
            posx(656.24, -138.42, 340.0, 0.0, 180.0, 0.0),
        ]

        r = 80.0
        loopN = 4

        dummy_index = 0
        for treeLocation in treeLocations:
            if dummy_index == 0:
                c = posx(treeLocation)
                p0 = copy.deepcopy(c)
                p0[1] -= r
                movel(p0, vel=30, acc=100)

                mp0 = copy.deepcopy(p0)

                for i in range(loopN):
                    p0[2] -= 10.0
                    movel(p0, vel=60, acc=100)
                    p0[2] += 10.0
                    movel(p0, vel=60, acc=100)
                    p0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 1.0) + (3.0 / 2.0) * pi
                    )
                    p0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 1.0) + (3.0 / 2.0) * pi
                    )
                    mp0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 0.5) + (3.0 / 2.0) * pi
                    )
                    mp0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 0.5) + (3.0 / 2.0) * pi
                    )
                    movec(mp0, p0, vel=60, acc=100)
            elif dummy_index == 1:
                c = posx(treeLocation)
                p0 = copy.deepcopy(c)
                p0[1] += r
                movel(p0, vel=30, acc=100)

                mp0 = copy.deepcopy(p0)

                for i in range(loopN - 1):
                    p0[2] -= 10.0
                    movel(p0, vel=60, acc=100)
                    p0[2] += 10.0
                    movel(p0, vel=60, acc=100)
                    p0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 1.0) + (1.0 / 2.0) * pi
                    )
                    p0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 1.0) + (1.0 / 2.0) * pi
                    )
                    mp0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 0.5) + (1.0 / 2.0) * pi
                    )
                    mp0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 0.5) + (1.0 / 2.0) * pi
                    )
                    movec(mp0, p0, vel=60, acc=100)
            else:
                c = posx(treeLocation)
                p0 = copy.deepcopy(c)
                p0[1] += r
                movel(p0, vel=30, acc=100)

                mp0 = copy.deepcopy(p0)

                for i in range(loopN):
                    p0[2] -= 10.0
                    movel(p0, vel=60, acc=100)
                    p0[2] += 10.0
                    movel(p0, vel=60, acc=100)
                    print(f"{p0[0]}, {p0[1]}, {p0[2]}")
                    p0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 1.0) + (1.0 / 2.0) * pi
                    )
                    p0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 1.0) + (1.0 / 2.0) * pi
                    )
                    mp0[0] = c[0] + r * cos(
                        (2.0 * pi / loopN) * (i + 0.5) + (1.0 / 2.0) * pi
                    )
                    mp0[1] = c[1] + r * sin(
                        (2.0 * pi / loopN) * (i + 0.5) + (1.0 / 2.0) * pi
                    )
                    movec(mp0, p0, vel=60, acc=100)
            info(f"===== 땅 다지기 작업: {dummy_index+1}개 완료 =====")
            dummy_index += 1

        wait(1)
        ## tool 복귀
        # 충돌 대비해서 목표 위치보다 수직으로 위인 지점으로 선형 이송 시키기
        ptb1, _ = get_current_posx()
        ptb1[2] += 100.0
        movel(ptb1, vel=60, acc=100)

        movej(j0, vel=60, acc=30)
        wait(1)
        movel([0, 0, 200, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)

        ##
        movej([-180, 0, 0, 0, 0, 0], mod=DR_FC_MOD_REL, vel=60, acc=20)

        # tool 꽂을 곳 위로 이동
        # movel(tool_up, vel=60, acc=100)
        pc, _ = get_current_posx()
        movel(
            [tool_up[0], tool_up[1], pc[2], pc[3], pc[4], pc[5]],
            vel=60,
            acc=100,
            mod=DR_MV_MOD_ABS,
        )
        movel(
            [tool_up[0], tool_up[1], tool_up[2], pc[3], pc[4], pc[5]],
            vel=60,
            acc=100,
            mod=DR_MV_MOD_ABS,
        )
        # tool 꽂기
        movel(tool_pos, vel=20, acc=100)
        # gripper 열기
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)
        # 좌표계 복귀
        set_ref_coord(DR_BASE)
        # 충돌 대비해서 위로 올리기
        pc, _ = get_current_posx()
        pc[2] += 100.0
        movel(pc, vel=60, acc=100)
        movel([100, 0, 0, 0, 0, 0], vel=60, acc=100, mod=DR_FC_MOD_REL)
        # home으로 돌아오기
        movej(j0, vel=20, acc=30)
        info("===== 땅 다지기 완료 =====")

    def step4():
        # TODO: 황규빈
        #--------------------------------------------------------------------------------------------------------------
        # 필요 함수 모음
        #--------------------------------------------------------------------------------------------------------------
        info("==== 물 주기 시작 ====")
        from DSR_ROBOT2 import set_digital_output, wait,ON,OFF  # 필요한 기능만 임포트
        from DSR_ROBOT2 import posx,movej, DR_MV_MOD_REL, movel, DR_BASE, DR_TOOL, get_current_posx
        from DSR_ROBOT2 import set_velj, set_velx, set_accj, set_accx
        set_velx(60)
        set_velj(60)
        set_accj(60)
        set_accx(60)

        def home(): #처음 위치로 이동
            i = 0
            JReady = [0, 0, 90, 0, 90, 0]
            print("move home")
            
            movej(JReady, vel = 100, acc=100)
            wait(1)
            
            i+=1

    #--------------------------------------------------------------------------------------------------------------        
        
        def grip():  # 그립
            
            print('grip')
            
            set_digital_output(1,ON)
            set_digital_output(2,OFF)
            
            wait(2)

    #--------------------------------------------------------------------------------------------------------------
    
        def release(): # 그립 풀기
            
            print('release')
            
            set_digital_output(2,ON)
            set_digital_output(1,OFF)
            
            wait(2)

    #--------------------------------------------------------------------------------------------------------------
    
        def up(): #올라가기
            
            print('move up')
            
            pos_rel = [0.0, 0.0, 110.0, 0, 0, 0]
            movel(pos_rel, ref=DR_BASE, mod=DR_MV_MOD_REL)

    #--------------------------------------------------------------------------------------------------------------
    
        def down(): # 내려가기
            
            pos_rel = [0.0, 0.0, 110.0, 0, 0, 0]
            movel(pos_rel, ref=DR_TOOL, mod=DR_MV_MOD_REL)

    #--------------------------------------------------------------------------------------------------------------
    
        def toolfix(): # 툴 위치로 이동
            print('tool fix')
            
            posx1 = posx([-387.950,-97.370,582.260,51.3,-178.47,47.02])
            movel(posx1,vel = 100,acc=100)

    #--------------------------------------------------------------------------------------------------------------
        
        def tree():  #묘목 위치로 이동
            
            print('tree')
            
            pos1 = posx([356.29, 58.87, 420, 90, 180, 90]) # Y값 20mm 적게 잡음
            movel(pos1,vel = 100,acc=100)
            
            pos2 = posx([356.29, 58.87, 420, 90, 205, 90]) # Y값 20mm 적게 잡음
            movel(pos2,vel = 100,acc=100)

    #--------------------------------------------------------------------------------------------------------------
    
        def back(): #뒤로 돌아가기
            print('back')
            pos_rel = [0, -450, 0, 0, 0, 0]
            movel(pos_rel,vel=60,acc=60, mod=DR_MV_MOD_REL)
            
    #--------------------------------------------------------------------------------------------------------------
        
        home() # 처음 위치로 이동

        release() #그립 풀기

        JReady1 = [-90, 0, 90, 0, 90, 0] # tool로 가기위한 경유 지점
        
        movej(JReady1)
            
        toolfix() #tool 위치로 이동
        
        down()
        
        grip() # 그립
        
        wait(1)

        up() 
    
        home() #처음 

        tree() #처음 묘목 위치로 이동
        
        print(1) 

        print('물주기 시작') # 물주기 시작
        
        movel([280,0,0,0,0,0],mod = DR_MV_MOD_REL)
        
        p2,_ = get_current_posx()

        p2[3:6] =[90,180,90] 
        
        movel(p2)
        
        movej([0,-10,20,0,15,90],mod=DR_MV_MOD_REL)
        
        p3,_ = get_current_posx()

        print(p3)

        print(2)
        
        movel([0,-200,0,0,0,0],mod=DR_MV_MOD_REL)
        
        print(3)
        
        movel([0,230,0,0,0,0],mod=DR_MV_MOD_REL)
        
        print(4)

        movel([0,250,0,0,0,0],mod=DR_MV_MOD_REL)

        pos2 = posx([356.29, 58.87, 420, 90, 205, 90]) 
        movel(pos2,vel = 100,acc=100)
        
        pos1 = posx([356.29, 58.87, 420, 90, 180, 90]) 
        movel(pos1,vel = 100,acc=100)                     # 물주기 끝

        home() #처음 위치로 이동

        movej(JReady1) #tool위치로 가기 전 경유 지점
        
        toolfix() # tool 위치로 이동
        
        down()
        
        release()
        
        up()
        
        home() # 처음 위치로 이동
        
        info("==== 물 주기 끝 ====")
    # === Play 부분 =========================================================
    # 초기 위치 home 으로 이동
    move_to_home()

    # Steps
    step1()
    step2()
    step3()
    step4()

    # 작업 마무리 하고 home 으로 이동
    move_to_home()

    # === Exit 부분 =========================================================
    exit()


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
