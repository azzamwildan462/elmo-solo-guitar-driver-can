g++ basic_ping.cpp -o ping_all -lElmoCan 
g++ basic_pdo.cpp -o pdo -lElmoCan 
g++ basic_torque_control.cpp -o torq -lElmoCan 
g++ basic_speed_control.cpp -o speed -lElmoCan 
g++ basic_configure.cpp -o configureElmo -lElmoCan 
g++ basic_inverse_kinematic.cpp -o inverseKinematic -lElmoCan -lm
g++ basic_inverse_kinematic_torque.cpp -o torqInversKinematic -lElmoCan -lm

g++ up_iface.cpp -o up_iface
