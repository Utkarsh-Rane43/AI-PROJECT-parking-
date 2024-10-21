import numpy as np
import heapq
import tkinter as tk
from tkinter import messagebox

# Define the parking layout
parking_layout = [
    ['A',1,2,3,'X',5,'B'],
    [7,'X',9,'X',11,'X',13],
    ['C',15,'X',17,'X',19,'D']
    # ['A', 1, 2, 3, 4, 5, 6, 7, 8, 'B'],
    # [9, 'X', 11, 12, 'X', 14, 15, 'X', 17, 18],
    # [19, 20, 21, 22, 23, 'X', 25, 26, 27, 28],
    # [29, 'X', 31, 32, 'X', 34, 35, 'X', 37, 38],
    # [39, 40, 'X', 42, 43, 'X', 45, 46, 47, 'X'],
    # [49, 'X', 51, 52, 'X', 54, 55, 'X', 57, 58],
    # [59, 60, 61, 'X', 63, 64, 65, 66, 'X', 68],
    # [69, 'C', 71, 72, 73, 74, 75, 76, 'D', 78]
]

# Define entry and exit points
entry_points = {'A': (0, 0), 'B': (0, 6), 'C': (2, 0), 'D': (2, 6)}
parking_spots = {(r, c): parking_layout[r][c] for r in range(len(parking_layout)) for c in range(len(parking_layout[r])) if isinstance(parking_layout[r][c], int)}
occupied_spots = {}
car_data_file = "car_data_astar.txt"  # File to store car data

def heuristic(a, b):
    """Calculate the heuristic for A* (Manhattan distance)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(start, goal, layout):
    """A* search algorithm to find the optimal path."""
    rows, cols = len(layout), len(layout[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, down, left, right
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < rows) and (0 <= neighbor[1] < cols):
                if isinstance(layout[neighbor[0]][neighbor[1]], str):  # Ignore entries and exits
                    continue
                
                tentative_g_score = g_score[current] + 1  # Distance to neighbor
                if neighbor in g_score and tentative_g_score >= g_score[neighbor]:
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found

def reconstruct_path(came_from, current):
    """Reconstruct the path from start to goal."""
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]  # Reverse the path

def find_nearest_parking_spot(start):
    """Find the nearest empty parking spot from the entry point."""
    empty_spots = [spot for spot in parking_spots.keys() if spot not in occupied_spots]
    if not empty_spots:
        return None
    
    nearest_spot = min(empty_spots, key=lambda x: heuristic(start, x))
    return nearest_spot

def park_car(car_number, entry_gate):
    """Park the car in the nearest available spot."""
    # Check if the car is already parked
    if car_number in occupied_spots.values():
        return None, "Car already parked."
    
    entry_point = entry_points[entry_gate]
    nearest_spot = find_nearest_parking_spot(entry_point)
    
    if nearest_spot:
        # Use A* algorithm to find the path
        path = astar(entry_point, nearest_spot, parking_layout)
        
        # Check if the path was found
        if path is None:
            return None, "No valid path to the parking spot."
        
        total_cost = len(path)  # Each block has a cost of 1 unit
        
        occupied_spots[nearest_spot] = car_number
        
        # Save car data and cost to the file
        with open(car_data_file, "a") as file:
            file.write(f"{car_number}, {path}, {nearest_spot}, Cost: {total_cost}\n")
        
        return nearest_spot, None
    else:
        return None, "No available parking spots."
def find_nearest_exit(car_spot):
    """Find the nearest exit point from the parking spot."""
    empty_spots = entry_points.values()
    
    nearest_exit = min(empty_spots, key=lambda x: heuristic(car_spot, x))
    return nearest_exit

def unpark_car(car_number):
    """Unpark the car based on the car number."""
    for spot, number in occupied_spots.items():
        if number == car_number:
            nearest_exit = find_nearest_exit(spot)  # Find the nearest exit point
            path = astar(spot, nearest_exit, parking_layout)  # Get the path to the exit
            
            del occupied_spots[spot]  # Free the parking spot
            
            # Save unpark data (including the exit path) to the file
            with open(car_data_file, "a") as file:
                file.write(f"{car_number} unparked from {spot} to exit at {nearest_exit}, Path: {path}\n")
            
            return spot, nearest_exit, path  # Return the freed spot and the nearest exit
    return None, None, None  # Car not found

def draw_parking_space(canvas):
    """Draw the parking layout on the canvas."""
    canvas.delete("all")  # Clear previous drawings
    for r in range(len(parking_layout)):
        for c in range(len(parking_layout[r])): 
            value = parking_layout[r][c]
            spot = (r, c)
            if isinstance(value, int):
                if spot in occupied_spots:  # Red with car number if occupied
                    color = "red"
                    display_value = occupied_spots[spot]
                else:  # Green if empty
                    color = "green"
                    display_value = value
            else:
                color = "lightgray"
                display_value = value
            x1, y1 = c * 50, r * 50
            x2, y2 = x1 + 50, y1 + 50
            canvas.create_rectangle(x1, y1, x2, y2, fill=color)
            canvas.create_text(x1 + 25, y1 + 25, text=display_value)

def on_park():
    """Handle parking a car."""
    car_number = car_number_entry.get().strip()
    entry_gate = entry_gate_entry.get().strip().upper()
    
    if entry_gate in entry_points:
        spot, error = park_car(car_number, entry_gate)
        if spot:
            messagebox.showinfo("Parking Success", f"Car {car_number} parked at {spot}.")
            draw_parking_space(canvas)
        else:
            messagebox.showerror("Parking Failed", error)
    else:
        messagebox.showerror("Invalid Entry Gate", "Please enter a valid entry gate (A, B, C, or D).")

def on_unpark():
    """Handle unparking a car."""
    car_number = car_number_entry.get().strip()
    
    freed_spot, nearest_exit, path = unpark_car(car_number)
    if freed_spot:
        messagebox.showinfo("Unparking Success", f"Car {car_number} has been unparked from {freed_spot} to exit at {nearest_exit}. Path: {path}")
        draw_parking_space(canvas)
    else:
        messagebox.showerror("Unparking Failed", "Car not found.")
# Create the main application window
root = tk.Tk()
root.title("Smart Parking System")

# Create a frame for inputs and buttons
frame = tk.Frame(root)
frame.pack()

# Car number entry
tk.Label(frame, text="Car Number:").grid(row=0, column=0)
car_number_entry = tk.Entry(frame)
car_number_entry.grid(row=0, column=1)

# Entry gate entry
tk.Label(frame, text="Entry Gate:").grid(row=1, column=0)
entry_gate_entry = tk.Entry(frame)
entry_gate_entry.grid(row=1, column=1)

# # Exit gate entry
# tk.Label(frame, text="Exit Gate:").grid(row=2, column=0)
# exit_gate_entry = tk.Entry(frame)
# exit_gate_entry.grid(row=2, column=1)

# Park button
park_button = tk.Button(frame, text="Park Car", command=on_park)
park_button.grid(row=3, column=0, pady=10)

# Unpark button
unpark_button = tk.Button(frame, text="Unpark Car", command=on_unpark)
unpark_button.grid(row=3, column=1, pady=10)

# Create a canvas to draw the parking layout
canvas = tk.Canvas(root, width=500, height=400, bg="white")
canvas.pack()

# Draw the initial parking space
draw_parking_space(canvas)

# Run the application
root.mainloop()
