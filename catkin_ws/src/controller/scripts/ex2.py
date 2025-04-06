#!/usr/bin/python3   

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # OBSTACLE
        self.obstacle_x = 3
        self.obstacle_y = 3
        self.radius = 0.5 + 0.2  # l'obstacle est de forme circulaire de diamètre 1u. Donc, 0.5 est le rayon de la tortue 
        # et 0.2 est un marge VISUELLE de sécurité pour éviter la collision.
        self.avoiding_obstacle = False

        # TURTLE ENTITY
        self.linear_speed = 2.0  # Si vous voulez que la tortue aille plus vite, augmentez cette valeur.
        # J'ai essayé la valeur 10, mais dans le cas de l'obstacle, il y a eu quelques problèmes.
        self.tolerance = 0.1
        self.angular_speed = self.linear_speed / self.radius

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # PROJECT ENTITY
        self.rate = rospy.Rate(10)

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def move2goal(self, x, y):
        while not rospy.is_shutdown():
            dx = x - self.current_x
            dy = y - self.current_y

            distance = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            angle_diff = self.normalize(angle_to_target - self.current_theta)

            if self.avoiding_obstacle and self.euclidean_distance(self.obstacle_x, self.obstacle_y) > self.radius + 0.2: # buffer because I saw one case where it triggerd too early.
                self.avoiding_obstacle = False

            # Vérifie si on est trop proche de l'obstacle
            if not self.avoiding_obstacle and self.euclidean_distance(self.obstacle_x, self.obstacle_y) < self.radius:
                rospy.loginfo("[Obstacle] dans la position: x: %.2f, y: %.2f", self.obstacle_x, self.obstacle_y)
                self.avoiding_obstacle = True
                self.circle_obstacle()
                continue

            cmd = Twist()
            if distance < self.tolerance:
                rospy.loginfo(f"[Succès] Arret à x: {x:.2f}, y: {y:.2f}")
                break

            if abs(angle_diff) > 0.2:
                cmd.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = min(self.linear_speed, distance)
                cmd.angular.z = 0.0


            self.vel_pub.publish(cmd)
            # Publish at the desired rate.
            self.rate.sleep()

        self.stop()

    def circle_obstacle(self):
        dx = self.current_x - self.obstacle_x
        dy = self.current_y - self.obstacle_y
        angle_from_obstacle = math.atan2(dy, dx)

        # Tangente à la trajectoire circulaire — correspond à une rotation de 90° autour du centre de l’obstacle
        tangent = self.normalize(angle_from_obstacle + math.pi / 2)

        # S'aligne avec la direction tangentielle pour commencer l’orbite
        while abs(self.normalize(tangent - self.current_theta)) > 0.1:
            cmd = Twist()
            cmd.angular.z = 0.5 * self.normalize(tangent - self.current_theta)
            self.vel_pub.publish(cmd)
            self.rate.sleep()

        # Effectue un demi-cercle autour de l'obstacle
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = (self.linear_speed / self.radius)

        # v = ω * r  → ω = v/r
        # Durée = distance / vitesse = (π * r) / v  → pour un demi-cercle (180°)
        duration = math.pi * self.radius / self.linear_speed
        t0 = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - t0 < duration:
            self.vel_pub.publish(cmd)
            self.rate.sleep()

        self.stop()

    def euclidean_distance(self, x1, y1, x2=None, y2=None):
        """Si x2, y2 ne sont pas fournis, calcule la distance entre la tortue et (x1, y1).
        Sinon, calcule entre deux points (x1, y1) et (x2, y2)."""
        if x2 is None or y2 is None:
            return math.hypot(self.current_x - x1, self.current_y - y1)
        return math.hypot(x1 - x2, y1 - y2)

    def normalize(self, angle):
        """Normalise l’angle dans l’intervalle [-π, π].
        Parfois, l’angle dépasse π. 
        Le modulo % (2π) ramène l’angle dans [0, 2π], puis on le décale vers [-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def stop(self):
        # mensagem default.
        self.vel_pub.publish(Twist())

    def run(self):
        positions = [0.75, 2.25, 3.75, 5.25, 6.75, 8.25, 9.75]
        for idx, j in enumerate(positions):
            # Pour les lignes d’indice pair, on avance de gauche à droite ; sinon de droite à gauche.
            if idx % 2 == 0:
                row = positions  
            else:
                row = reversed(positions)
            for i in row:
                if self.euclidean_distance(i, j, self.obstacle_x, self.obstacle_y) < self.radius :
                    rospy.loginfo(f"[Ignoré] Point proche de l'obstacle: x={i:.2f}, y={j:.2f}")
                    continue
                self.move2goal(i, j)

if __name__ == '__main__':
    try:
        TurtleController().run()
    except rospy.ROSInterruptException:
        pass
