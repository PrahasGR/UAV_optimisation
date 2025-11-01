#!/usr/bin/env python3
"""
Frontend GUI for GSDA vs Distributed GA Drone Positioning
Visualizes drone positions, user locations, and performance metrics
"""

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import random
from threading import Thread

# Import your backend functions
try:
    from backend.src.gsda import (
        GSDA_analytic, distributed_GA, grid_centers, 
        user_assignments, total_capacity, zone_side
    )
except ImportError:
    print("Warning: Could not import backend modules. Using mock functions.")
    # Mock functions for testing if import fails
    zone_side = 600.0
    def grid_centers(rows, cols, zone=600, origin=(0,0)):
        centers = []
        for r in range(rows):
            for c in range(cols):
                centers.append((origin[0]+(c+0.5)*zone, origin[1]+(r+0.5)*zone))
        return centers
    def GSDA_analytic(init, users, rows, cols, **kwargs):
        return init, 475.0
    def distributed_GA(init, users, rows, cols, **kwargs):
        return init, 443.0
    def total_capacity(pos, users, rows, cols):
        return 450.0


class DronePositioningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Positioning: GSDA vs Distributed GA")
        self.root.geometry("1400x900")
        self.root.configure(bg='#f0f0f0')
        
        # Data storage
        self.users = []
        self.gsda_positions = []
        self.ga_positions = []
        self.gsda_capacity = 0
        self.ga_capacity = 0
        self.rows = 3
        self.cols = 3
        self.num_users = 100
        self.running = False
        
        self.setup_ui()
        
    def setup_ui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Title
        title = ttk.Label(main_frame, text="UAV-Assisted Communication Network Optimization", 
                         font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Control Panel
        control_frame = ttk.LabelFrame(main_frame, text="Configuration", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # Grid size
        ttk.Label(control_frame, text="Grid Rows:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.rows_var = tk.IntVar(value=3)
        ttk.Spinbox(control_frame, from_=2, to=5, textvariable=self.rows_var, width=10).grid(row=0, column=1, pady=2)
        
        ttk.Label(control_frame, text="Grid Cols:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.cols_var = tk.IntVar(value=3)
        ttk.Spinbox(control_frame, from_=2, to=5, textvariable=self.cols_var, width=10).grid(row=1, column=1, pady=2)
        
        # Number of users
        ttk.Label(control_frame, text="Users:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.users_var = tk.IntVar(value=100)
        ttk.Spinbox(control_frame, from_=20, to=500, textvariable=self.users_var, width=10, increment=10).grid(row=2, column=1, pady=2)
        
        # Algorithm parameters
        ttk.Label(control_frame, text="GSDA Iterations:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.gsda_iter_var = tk.IntVar(value=10)
        ttk.Spinbox(control_frame, from_=3, to=20, textvariable=self.gsda_iter_var, width=10).grid(row=3, column=1, pady=2)
        
        ttk.Label(control_frame, text="GA Iterations:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.ga_iter_var = tk.IntVar(value=3)
        ttk.Spinbox(control_frame, from_=1, to=10, textvariable=self.ga_iter_var, width=10).grid(row=4, column=1, pady=2)
        
        # Random seed option
        self.random_seed_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(control_frame, text="Use Random Seed (different results each run)", 
                       variable=self.random_seed_var).grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # Run button
        self.run_btn = ttk.Button(control_frame, text="Run Optimization", command=self.run_optimization)
        self.run_btn.grid(row=6, column=0, columnspan=2, pady=15)
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.status_var, foreground='blue')
        status_label.grid(row=7, column=0, columnspan=2)
        
        # Results Panel
        results_frame = ttk.LabelFrame(main_frame, text="Results", padding="10")
        results_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        ttk.Label(results_frame, text="GSDA Capacity:", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W)
        self.gsda_cap_var = tk.StringVar(value="-- bps/Hz")
        ttk.Label(results_frame, textvariable=self.gsda_cap_var, foreground='green').grid(row=0, column=1, sticky=tk.W)
        
        ttk.Label(results_frame, text="GA Capacity:", font=('Arial', 10, 'bold')).grid(row=1, column=0, sticky=tk.W)
        self.ga_cap_var = tk.StringVar(value="-- bps/Hz")
        ttk.Label(results_frame, textvariable=self.ga_cap_var, foreground='orange').grid(row=1, column=1, sticky=tk.W)
        
        ttk.Label(results_frame, text="Improvement:", font=('Arial', 10, 'bold')).grid(row=2, column=0, sticky=tk.W)
        self.improvement_var = tk.StringVar(value="-- %")
        ttk.Label(results_frame, textvariable=self.improvement_var, foreground='red').grid(row=2, column=1, sticky=tk.W)
        
        # Visualization area
        viz_frame = ttk.Frame(main_frame)
        viz_frame.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(viz_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab 1: Drone positions
        self.pos_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.pos_frame, text="Drone Positions")
        
        # Tab 2: Performance comparison
        self.perf_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.perf_frame, text="Performance Metrics")
        
        # Tab 3: Height distribution
        self.height_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.height_frame, text="Height Analysis")
        
        # Configure grid weights
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
    def generate_users(self):
        """Generate random user positions"""
        total_side = zone_side * self.rows
        self.users = []
        for _ in range(self.num_users):
            x = random.uniform(0, total_side)
            y = random.uniform(0, total_side)
            self.users.append((x, y))
    
    def run_optimization(self):
        """Run both algorithms in a separate thread"""
        if self.running:
            messagebox.showwarning("Running", "Optimization already in progress!")
            return
        
        self.running = True
        self.run_btn.config(state='disabled')
        self.status_var.set("Running optimization...")
        
        thread = Thread(target=self.run_algorithms)
        thread.start()
    
    def run_algorithms(self):
        """Execute GSDA and GA algorithms"""
        try:
            # Get parameters
            self.rows = self.rows_var.get()
            self.cols = self.cols_var.get()
            self.num_users = self.users_var.get()
            gsda_iters = self.gsda_iter_var.get()
            ga_iters = self.ga_iter_var.get()
            
            # Generate users and initial positions
            if not self.random_seed_var.get():
                # Use fixed seed for reproducibility
                random.seed(42)
                np.random.seed(42)
            # else: truly random results each run
            self.generate_users()
            
            centers = grid_centers(self.rows, self.cols)
            init_positions = [(cx, cy, 100.0) for (cx, cy) in centers]
            
            # Run GSDA
            self.root.after(0, lambda: self.status_var.set("Running GSDA..."))
            self.gsda_positions, self.gsda_capacity = GSDA_analytic(
                init_positions, self.users, self.rows, self.cols,
                K_outer=gsda_iters, K_sub=(150, 150, 400),
                s_vals=(3000.0, 3000.0, 900.0), verbose=False
            )
            
            # Run GA
            self.root.after(0, lambda: self.status_var.set("Running Distributed GA..."))
            self.ga_positions, self.ga_capacity = distributed_GA(
                init_positions, self.users, self.rows, self.cols,
                KG=ga_iters, KpG=1, ga_pop=10, ga_gen=15
            )
            
            # Update UI
            self.root.after(0, self.update_results)
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Error", f"Optimization failed: {str(e)}"))
            self.root.after(0, lambda: self.status_var.set("Error occurred"))
        finally:
            self.running = False
            self.root.after(0, lambda: self.run_btn.config(state='normal'))
    
    def update_results(self):
        """Update result displays and visualizations"""
        self.gsda_cap_var.set(f"{self.gsda_capacity:.2f} bps/Hz")
        self.ga_cap_var.set(f"{self.ga_capacity:.2f} bps/Hz")
        
        improvement = ((self.gsda_capacity - self.ga_capacity) / self.ga_capacity) * 100
        self.improvement_var.set(f"{improvement:.2f} %")
        
        self.status_var.set("Optimization complete!")
        
        # Create visualizations
        self.plot_positions()
        self.plot_performance()
        self.plot_height_analysis()
    
    def plot_positions(self):
        """Plot drone and user positions"""
        # Clear previous plot
        for widget in self.pos_frame.winfo_children():
            widget.destroy()
        
        fig = Figure(figsize=(12, 5), dpi=100)
        
        # GSDA subplot
        ax1 = fig.add_subplot(121)
        self.plot_scenario(ax1, self.gsda_positions, "GSDA", 'green')
        
        # GA subplot
        ax2 = fig.add_subplot(122)
        self.plot_scenario(ax2, self.ga_positions, "Distributed GA", 'orange')
        
        fig.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, master=self.pos_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def plot_scenario(self, ax, drone_positions, title, color):
        """Plot a single scenario"""
        total_side = zone_side * self.rows
        
        # Plot zones
        for i in range(self.rows + 1):
            ax.axhline(y=i * zone_side, color='lightgray', linestyle='--', linewidth=0.5)
        for j in range(self.cols + 1):
            ax.axvline(x=j * zone_side, color='lightgray', linestyle='--', linewidth=0.5)
        
        # Plot users
        users_x = [u[0] for u in self.users]
        users_y = [u[1] for u in self.users]
        ax.scatter(users_x, users_y, c='blue', marker='.', s=10, alpha=0.5, label='Users')
        
        # Plot drones
        drones_x = [d[0] for d in drone_positions]
        drones_y = [d[1] for d in drone_positions]
        ax.scatter(drones_x, drones_y, c=color, marker='^', s=200, 
                  edgecolors='black', linewidths=1.5, label='UAVs', zorder=5)
        
        # Add drone labels
        for i, (x, y, h) in enumerate(drone_positions):
            ax.annotate(f'D{i}\n{h:.0f}m', (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=8, fontweight='bold')
        
        ax.set_xlim(-50, total_side + 50)
        ax.set_ylim(-50, total_side + 50)
        ax.set_xlabel('X (meters)', fontsize=10)
        ax.set_ylabel('Y (meters)', fontsize=10)
        ax.set_title(f'{title}\nCapacity: {self.gsda_capacity if color=="green" else self.ga_capacity:.2f} bps/Hz', 
                    fontsize=11, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
    
    def plot_performance(self):
        """Plot performance comparison metrics"""
        for widget in self.perf_frame.winfo_children():
            widget.destroy()
        
        fig = Figure(figsize=(12, 5), dpi=100)
        
        # Capacity comparison bar chart
        ax1 = fig.add_subplot(121)
        algorithms = ['GSDA', 'Distributed GA']
        capacities = [self.gsda_capacity, self.ga_capacity]
        colors = ['green', 'orange']
        bars = ax1.bar(algorithms, capacities, color=colors, alpha=0.7, edgecolor='black')
        ax1.set_ylabel('Total Capacity (bps/Hz)', fontsize=11)
        ax1.set_title('Algorithm Comparison', fontsize=12, fontweight='bold')
        ax1.grid(axis='y', alpha=0.3)
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f}', ha='center', va='bottom', fontweight='bold')
        
        # Per-drone capacity
        ax2 = fig.add_subplot(122)
        drone_ids = list(range(len(self.gsda_positions)))
        x = np.arange(len(drone_ids))
        width = 0.35
        
        # Calculate per-drone contributions (simplified)
        gsda_per = [self.gsda_capacity / len(self.gsda_positions)] * len(self.gsda_positions)
        ga_per = [self.ga_capacity / len(self.ga_positions)] * len(self.ga_positions)
        
        ax2.bar(x - width/2, gsda_per, width, label='GSDA', color='green', alpha=0.7)
        ax2.bar(x + width/2, ga_per, width, label='GA', color='orange', alpha=0.7)
        ax2.set_xlabel('Drone ID', fontsize=11)
        ax2.set_ylabel('Avg Capacity per Drone (bps/Hz)', fontsize=11)
        ax2.set_title('Per-Drone Performance', fontsize=12, fontweight='bold')
        ax2.set_xticks(x)
        ax2.set_xticklabels([f'D{i}' for i in drone_ids])
        ax2.legend()
        ax2.grid(axis='y', alpha=0.3)
        
        fig.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, master=self.perf_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def plot_height_analysis(self):
        """Plot height distribution and analysis"""
        for widget in self.height_frame.winfo_children():
            widget.destroy()
        
        fig = Figure(figsize=(12, 5), dpi=100)
        
        # Height distribution
        ax1 = fig.add_subplot(121)
        gsda_heights = [d[2] for d in self.gsda_positions]
        ga_heights = [d[2] for d in self.ga_positions]
        
        x = np.arange(len(gsda_heights))
        width = 0.35
        ax1.bar(x - width/2, gsda_heights, width, label='GSDA', color='green', alpha=0.7)
        ax1.bar(x + width/2, ga_heights, width, label='GA', color='orange', alpha=0.7)
        ax1.set_xlabel('Drone ID', fontsize=11)
        ax1.set_ylabel('Height (meters)', fontsize=11)
        ax1.set_title('Drone Height Distribution', fontsize=12, fontweight='bold')
        ax1.set_xticks(x)
        ax1.set_xticklabels([f'D{i}' for i in range(len(gsda_heights))])
        ax1.legend()
        ax1.grid(axis='y', alpha=0.3)
        ax1.set_ylim(0, 110)
        
        # Statistics
        ax2 = fig.add_subplot(122)
        ax2.axis('off')
        
        stats_text = f"""
        Height Statistics
        
        GSDA:
          Mean Height: {np.mean(gsda_heights):.2f} m
          Std Dev: {np.std(gsda_heights):.2f} m
          Min: {np.min(gsda_heights):.2f} m
          Max: {np.max(gsda_heights):.2f} m
        
        Distributed GA:
          Mean Height: {np.mean(ga_heights):.2f} m
          Std Dev: {np.std(ga_heights):.2f} m
          Min: {np.min(ga_heights):.2f} m
          Max: {np.max(ga_heights):.2f} m
        
        Analysis:
          GSDA tends to optimize heights near the upper
          bound (100m) for better coverage and LoS
          probability.
          
          GA shows more variation, exploring diverse
          height configurations.
        """
        
        ax2.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
                verticalalignment='center')
        
        fig.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, master=self.height_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)


def main():
    root = tk.Tk()
    app = DronePositioningGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()