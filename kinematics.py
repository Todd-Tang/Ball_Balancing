import math
import time

class Kinematics:
    def __init__(self):
        """Initialize the kinematics class"""
        # Member lengths (L3, L2, L1, L0)
        self.L = [0.065, 0.08, 0.075, 0.15]
        
    def inv_kinematics(self, n, Pz):
        """Calculate theta angles for the robot based on the normal vector n and the height Pz"""
        
        L = self.L

        # Find mean values for the x and z coordinates of the leg "knee" joint across 3 legs
        A = (L[0]+L[1])/Pz 
        B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
        C = A**2+1
        D = 2*(A*B-(L[0]+L[1]))
        E = B**2+(L[0]+L[1])**2-L[2]**2

        
        Pmx = (-D+math.sqrt(D**2-4*C*E))/(2*C) # Apply quadratic formula to find the x position of the leg "knee" joint
        Pmz = math.sqrt(L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2)
            # Pythagorean theorem to find the z position of the leg "knee" joint 

        #Inverse kinematics for the angle of servo a
        a_m_x = (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(n[2])
        a_m_y = 0
        a_m_z = Pz + (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(-n[0])
        #A_m = [a,b,c]
        A_m = [a_m_x, a_m_y, a_m_z]
        A = (L[0]-A_m[0])/A_m[2]
        B = (A_m[0]**2+A_m[1]**2+A_m[2]**2-L[2]**2-L[0]**2+L[1]**2)/(2*A_m[2])
        C = A**2+1
        D = 2*(A*B-L[0])
        E = B**2+L[0]**2-L[1]**2
        ax = (-D+math.sqrt(D**2-4*C*E))/(2*C)
        ay = 0
        az = math.sqrt(L[1]**2-ax**2+2*L[0]*ax-L[0]**2)

        if (a_m_z < Pmz):
            az = -az
        A_2 = [ax, ay, az]
        
        theta_a = 90 - math.degrees(math.atan2(A_2[0]-L[0], A_2[2]))

        b_m_x = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        b_m_y = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[2])
        b_m_z = Pz + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[1]+n[0])
        B_m = [b_m_x, b_m_y, b_m_z]

        A = -(B_m[0]+math.sqrt(3)*B_m[1]+2*L[0])/B_m[2]
        B = (B_m[0]**2+B_m[1]**2+B_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*B_m[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        x = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        y = math.sqrt(3)*x
        z = math.sqrt(L[1]**2-4*x**2-4*L[0]*x-L[0]**2)

        if (b_m_z < Pmz):
            z = -z

        B_2 = [x, y, z]
        
        theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0]**2+B_2[1]**2)-L[0], B_2[2]))

        c_m_x = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        c_m_y = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[2])
        c_m_z = Pz + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[1]+n[0])
        C_m = [c_m_x, c_m_y, c_m_z]

        A = -(C_m[0]-math.sqrt(3)*C_m[1]+2*L[0])/C_m[2]
        B = (C_m[0]**2+C_m[1]**2+C_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*C_m[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        x = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        y = -math.sqrt(3)*x
        z = math.sqrt(L[1]**2-4*x**2-4*L[0]*x-L[0]**2)

        if (c_m_z < Pmz):
            z = -z

        C_2 = [x, y, z]
        
        theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0]**2+C_2[1]**2)-L[0], C_2[2]))
        thetas = [theta_a, theta_b, theta_c]
        return thetas
