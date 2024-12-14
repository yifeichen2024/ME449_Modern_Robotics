from IPython.display import display
import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):

    positions = [] 
    linear_errors = []  
    angular_errors = []  
    
    thetalist = np.array(thetalist0).copy()
    iter_thetas = np.array(thetalist.copy())
    i = 0
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
    
    eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

    err = eomg_act > eomg or ev_act > ev
    
    T_act = mr.FKinBody(M, Blist, thetalist)

    print(" Iteration %d:\n\n \
        Joint Vector:\n \
        %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
        SE(3) End-Effector Configuration:\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n\n \
        Error Twist Vb:\n \
        %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
        Angular Error Magnitude || omega_b ||: %6.5f\n \
        Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
        T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
        T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))

    while err:

        i = i + 1
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        
        # prevent the joint vector over joint limit 
        thetalist = np.arctan2(np.sin(thetalist), np.cos(thetalist))

        iter_thetas = np.vstack([iter_thetas,thetalist])

        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))

        T_act = mr.FKinBody(M, Blist, thetalist)

        eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

        err = eomg_act > eomg or ev_act > ev

        # Store the position vector 
        positions.append([T_act[0, 3], T_act[1, 3], T_act[2, 3]])
        # Store linear errors 
        linear_errors.append(ev_act)
        # Store angular errors 
        angular_errors.append(eomg_act)

        print(" Iteration %d:\n\n \
                Joint Vector:\n \
                %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
                SE(3) End-Effector Configuration:\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n\n \
                Error Twist Vb:\n \
                %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
                Angular Error Magnitude || omega_b ||: %6.5f\n \
                Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
                T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
                T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))
    


    f = open("long_iterates.csv", "w") 

    for i in range(len(iter_thetas)):
        output = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f\n" % (iter_thetas[i,0], iter_thetas[i,1], iter_thetas[i,2], 
                                                                        iter_thetas[i,3], iter_thetas[i,4], iter_thetas[i,5])
        f.write(output)

    f.close()

    return iter_thetas, not err, positions, linear_errors, angular_errors


def plot_results(positions_s, linear_errors_s, angular_errors_s, positions_l, linear_errors_l, angular_errors_l):
    # 1. 绘制末端执行器位置的 3D 轨迹图
    x_positions_s = [pos[0] for pos in positions_s]
    y_positions_s = [pos[1] for pos in positions_s]
    z_positions_s = [pos[2] for pos in positions_s]

    x_positions_l = [pos[0] for pos in positions_l]
    y_positions_l = [pos[1] for pos in positions_l]
    z_positions_l = [pos[2] for pos in positions_l]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.plot(x_positions_s, y_positions_s, z_positions_s, marker='o', linestyle='-',linewidth = 3, color = 'k', label = 'Short')
    plt.plot(x_positions_l, y_positions_l, z_positions_l, marker='o', linestyle='--', color = 'b', label = 'Long')
    # 标注起点和终点
    ax.scatter(x_positions_s[0], y_positions_s[0], z_positions_s[0], color='g', marker='o', s=100, label='Start Point')  # 起点标记为绿色圈
    ax.scatter(x_positions_l[0], y_positions_l[0], z_positions_l[0], color='g', marker='o', s=100, label='Start Point')  # 起点标记为绿色圈
    ax.scatter(x_positions_s[-1], y_positions_s[-1], z_positions_s[-1], color='r', marker='x', s=100, label='End Point')  # 终点标记为红色叉

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('End-Effector Position at Each Iteration')
    plt.legend()
    plt.show()

    # 2. 绘制线性误差大小随迭代次数变化的图
    plt.figure()
    plt.plot(range(len(linear_errors_s)), linear_errors_s, marker='o', linestyle='-', color = 'k', label = 'Short')
    plt.plot(range(len(linear_errors_l)), linear_errors_l, marker='o', linestyle='--', color = 'b', label = 'Long')
    plt.xlabel('Iteration Number')
    plt.ylabel('Linear Error Magnitude ||v_b|| (m)')
    plt.title('Linear Error Magnitude vs. Iteration Number')
    plt.legend()
    plt.grid(True)
    plt.show()

    # 3. 绘制角误差大小随迭代次数变化的图
    plt.figure()
    plt.plot(range(len(angular_errors_s)), angular_errors_s, marker='o', linestyle='-', color = 'k', label = 'Short')
    plt.plot(range(len(linear_errors_l)), linear_errors_l, marker='o', linestyle='--', color = 'b', label = 'Long')
    plt.xlabel('Iteration Number')
    plt.ylabel('Angular Error Magnitude ||omega_b|| (rad)')
    plt.title('Angular Error Magnitude vs. Iteration Number')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    H1 = .089 # 1519
    H2 = .095 #08535
    W1 = .109 # 11235
    W2 = .082
    L1 = .425 # 24365
    L2 = .392 # 21325
    
    B1 = np.array([0,1,0,W1+W2,0,L1+L2])
    B2 = np.array([0,0,1,H2,-L1-L2, 0])
    B3 = np.array([0,0,1,H2,-L2,0])
    B4 = np.array([0,0,1,H2,0,0])
    B5 = np.array([0,-1,0,-W2,0,0])
    B6 = np.array([0,0,1,0,0,0])
    Blist = np.array([B1,B2,B3,B4,B5,B6])
    Blist = Blist.T

    
    thetalist0_short = np.array([-2, -1.5, -1.2, 2.4, 1.4, 1.2]) # np.array([-2.5536, -2.0013, -1.6068, 2.2761, 1.4830, 0.9889]) 
    thetalist0_long = np.array([-2.5601, -1.0485, -1.8119, -0.4478, 2.9107, -2.0351]) #np.array([1.707,-1.578,0,-1.514,-0.032,1.514])


    # M
    M = np.array([[-1, 0, 0, L1+L2], 
                  [0, 0, 1, W1+W2], 
                  [0, 1, 0, H1-H2],
                  [0,0,0,1]])
    
    T = np.array([[1, 0, 0, 0.3],
                  [0, 1, 0, 0.3],
                  [0, 0, 1, 0.4],
                  [0, 0, 0, 1]])

    eomg = 0.001
    ev = 0.0001

    iter_thetas_s, err_s, positions_s, linear_errors_s, angular_errors_s = IKinBodyIterates(Blist,M,T,thetalist0_short,eomg,ev)
    iter_thetas_l, err_short_l, positions_l, linear_errors_l, angular_errors_l = IKinBodyIterates(Blist,M,T,thetalist0_long,eomg,ev)

    # plot result 
    plot_results(positions_s, linear_errors_s, angular_errors_s, positions_l, linear_errors_l, angular_errors_l)

