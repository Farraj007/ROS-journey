import pygame
import math

# Initialize Pygame
pygame.init()

# Set up the window dimensions
window_width, window_height = 800, 600
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption('Steering Wheel Visualization')

# Load steering wheel image
steering_wheel_image = pygame.image.load('teleop_gui_pkg/scripts/steering.png').convert_alpha()  # Replace 'steering_wheel.png' with your image file

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get steering angle reading from ROS2 (replace this with your actual ROS2 reading)
    steering_angle = 10  # Replace with your actual ROS2 message retrieval

    # Clear the window
    window.fill(WHITE)

    # Draw the rotated steering wheel image
    rotated_steering_wheel = pygame.transform.rotate(steering_wheel_image, -steering_angle)  # Negative angle for clockwise rotation
    window.blit(rotated_steering_wheel, ((window_width - rotated_steering_wheel.get_width()) // 2, (window_height - rotated_steering_wheel.get_height()) // 2))

    # Draw the steering angle indicator
    indicator_length = 100
    indicator_thickness = 5
    indicator_color = BLACK
    angle_in_radians = math.radians(steering_angle)
    indicator_end_point = (
        window_width // 2 + int(indicator_length * math.cos(angle_in_radians)),
        window_height // 2 - int(indicator_length * math.sin(angle_in_radians))
    )
    pygame.draw.line(window, indicator_color, (window_width // 2, window_height // 2), indicator_end_point, indicator_thickness)

    # Update the display
    pygame.display.flip()

# Quit Pygame properly when the loop exits
pygame.quit()
