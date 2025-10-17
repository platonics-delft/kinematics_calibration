import argparse
import os
import yaml
import shutil
import sys
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from yourdfpy import URDF

from calibrate_fk.parameter_optimizer import ParameterOptimizer
from calibrate_fk.utils import check_urdf_path, check_data_path, remove_outliers, evaluate_model


def plot_ablation_box_plots(results_dict: dict, initial_stats: dict, offset_distance: float, latex: bool = False):
    """
    Create beautiful violin plots using seaborn for ablation study comparing different optimization scenarios
    """
    # Set seaborn style
    sns.set_style("whitegrid")
    sns.set_context("paper", font_scale=1.2)
    
    if latex:
        plt.rcParams.update({
            "pgf.texsystem": "pdflatex",
            "text.usetex": False,
            "font.family": "serif",
            "pgf.rcfonts": False,
        })
    else:
        # Use seaborn's default font settings
        sns.set_style("whitegrid", {"font.family": "sans-serif"})
    
    # Prepare data for seaborn
    consistency_data = []
    distortion_data = []
    
    # Add initial/baseline performance first
    initial_consistency = [stat["std_dev_fk"] for stat in initial_stats.values()]
    initial_distortion = [stat["distance_error"] for stat in initial_stats.values()]
    
    # Create dataframes for seaborn
    for value in initial_consistency:
        consistency_data.append({'Scenario': 'Before\nTraining', 'Consistency [m]': value})
    for value in initial_distortion:
        distortion_data.append({'Scenario': 'Before\nTraining', 'Distortion [m]': value})
    
    # Add optimized scenarios
    for scenario_name, stats in results_dict.items():
        consistency_values = [stat["std_dev_fk"] for stat in stats.values()]
        distortion_values = [stat["distance_error"] for stat in stats.values()]
        
        for value in consistency_values:
            consistency_data.append({'Scenario': scenario_name, 'Consistency [m]': value})
        for value in distortion_values:
            distortion_data.append({'Scenario': scenario_name, 'Distortion [m]': value})
    
    # Convert to DataFrames
    df_consistency = pd.DataFrame(consistency_data)
    df_distortion = pd.DataFrame(distortion_data)
    
    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    
    # Define a beautiful color palette
    colors = ['#2E2E2E', '#3498DB', '#2ECC71', '#E74C3C', '#F39C12']  # Dark gray for initial, then vibrant colors
    palette = sns.color_palette(colors[:len(df_consistency['Scenario'].unique())])

    # Consistency box plot with seaborn
    sns.boxplot(data=df_consistency, x='Scenario', y='Consistency [m]', 
                palette=palette, ax=ax1, linewidth=2)

    # Add individual points to show data distribution
    sns.stripplot(data=df_consistency, x='Scenario', y='Consistency [m]', 
                  color='white', alpha=0.7, size=4, ax=ax1, edgecolor='black', linewidth=0.5)

    # Customize consistency plot
    ax1.set_yscale('log')
    ax1.set_ylabel('Consistency σ [m]', fontsize=16, fontweight='bold')
    # ax1.set_xlabel('Optimization Scenario', fontsize=16, fontweight='bold')
    ax1.set_title('Consistency Comparison - Ablation Study', fontsize=18, fontweight='bold', pad=20)
    ax1.tick_params(axis='both', which='major', labelsize=12)
    ax1.tick_params(axis='x', rotation=90, labelsize=20)
    
    # Add grid for better readability
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_axisbelow(True)

    # Distortion box plot with seaborn
    sns.boxplot(data=df_distortion, x='Scenario', y='Distortion [m]', 
                palette=palette, ax=ax2, linewidth=2)

    # Add individual points to show data distribution
    sns.stripplot(data=df_distortion, x='Scenario', y='Distortion [m]', 
                  color='white', alpha=0.7, size=4, ax=ax2, edgecolor='black', linewidth=0.5)
    
    # Customize distortion plot
    ax2.set_yscale('log')
    ax2.set_ylabel('Distortion ε [m]', fontsize=16, fontweight='bold')
    # ax2.set_xlabel('Optimization Scenario', fontsize=16, fontweight='bold')
    ax2.set_title('Distortion Comparison - Ablation Study', fontsize=18, fontweight='bold', pad=20)
    ax2.tick_params(axis='both', which='major', labelsize=12)
    ax2.tick_params(axis='x', rotation=90, labelsize=20)
    
    # Add grid for better readability
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_axisbelow(True)
    
    # Remove top and right spines for cleaner look
    for ax in [ax1, ax2]:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_linewidth(1.5)
        ax.spines['bottom'].set_linewidth(1.5)
    
    # Adjust layout
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.2)  # Add extra space at bottom for vertical labels
    
    # Add subtle background color
    fig.patch.set_facecolor('white')
    
    return fig

def print_ablation_statistics(results_dict: dict, initial_stats: dict):
    """
    Print detailed statistics for each optimization scenario including initial baseline with nice formatting
    """
    print("\n" + "="*90)
    print("🔬 ABLATION STUDY RESULTS".center(90))
    print("="*90)
    
    # Print initial baseline first
    initial_consistency = [stat["std_dev_fk"] for stat in initial_stats.values()]
    initial_distortion = [stat["distance_error"] for stat in initial_stats.values()]
    
    initial_consistency_median = np.median(initial_consistency)
    initial_distortion_median = np.median(initial_distortion)
    
    print(f"\n📊 INITIAL MODEL (Before Training):")
    print(f"   Consistency (σ): {initial_consistency_median:.2e} m (median) | {np.mean(initial_consistency):.2e} m (mean)")
    print(f"   Distortion  (ε): {initial_distortion_median:.2e} m (median) | {np.mean(initial_distortion):.2e} m (mean)")
    
    print(f"\n{'-'*90}")
    print("🚀 OPTIMIZATION SCENARIOS:".center(90))
    print(f"{'-'*90}")
    
    # Define emojis for each scenario
    scenario_emojis = ["🎯", "⚡", "🔧", "🌟"]
    
    for i, (scenario_name, stats) in enumerate(results_dict.items()):
        consistency_values = [stat["std_dev_fk"] for stat in stats.values()]
        distortion_values = [stat["distance_error"] for stat in stats.values()]
        
        consistency_median = np.median(consistency_values)
        distortion_median = np.median(distortion_values)
        
        # Calculate improvement relative to initial baseline
        consistency_improvement = (1 - consistency_median/initial_consistency_median) * 100
        distortion_improvement = (1 - distortion_median/initial_distortion_median) * 100
        
        emoji = scenario_emojis[i] if i < len(scenario_emojis) else "🔍"
        scenario_clean = scenario_name.replace('\n', ' ')
        
        print(f"\n{emoji} {scenario_clean}:")
        print(f"   Consistency (σ): {consistency_median:.2e} m (median) | {np.mean(consistency_values):.2e} m (mean)")
        print(f"   Distortion  (ε): {distortion_median:.2e} m (median) | {np.mean(distortion_values):.2e} m (mean)")
        
        # Color code improvements
        consistency_color = "✅" if consistency_improvement > 0 else "❌"
        distortion_color = "✅" if distortion_improvement > 0 else "❌"
        
        print(f"   📈 Improvements: {consistency_color} Consistency: {consistency_improvement:+.1f}% | "
              f"{distortion_color} Distortion: {distortion_improvement:+.1f}%")
    
    # Summary statistics
    print(f"\n{'-'*90}")
    print("📋 SUMMARY:".center(90))
    print(f"{'-'*90}")
    
    all_consistency_improvements = []
    all_distortion_improvements = []
    
    for scenario_name, stats in results_dict.items():
        consistency_values = [stat["std_dev_fk"] for stat in stats.values()]
        distortion_values = [stat["distance_error"] for stat in stats.values()]
        
        consistency_median = np.median(consistency_values)
        distortion_median = np.median(distortion_values)
        
        consistency_improvement = (1 - consistency_median/initial_consistency_median) * 100
        distortion_improvement = (1 - distortion_median/initial_distortion_median) * 100
        
        all_consistency_improvements.append(consistency_improvement)
        all_distortion_improvements.append(distortion_improvement)
    
    best_consistency_idx = np.argmax(all_consistency_improvements)
    best_distortion_idx = np.argmax(all_distortion_improvements)
    
    scenario_names = list(results_dict.keys())
    best_consistency_scenario = scenario_names[best_consistency_idx].replace('\n', ' ')
    best_distortion_scenario = scenario_names[best_distortion_idx].replace('\n', ' ')
    
    print(f"   🏆 Best Consistency Improvement: {best_consistency_scenario} ({all_consistency_improvements[best_consistency_idx]:+.1f}%)")
    print(f"   🏆 Best Distortion Improvement:  {best_distortion_scenario} ({all_distortion_improvements[best_distortion_idx]:+.1f}%)")
    print("="*90)


def main():
    argument_parser = argparse.ArgumentParser(description='Run parameter optimizer ablation study comparing different optimization scenarios')
    argument_parser.add_argument("--model", "-m", help="Name of the urdf stored in the urdf folder.", required=True)
    argument_parser.add_argument("--data", "-tr", type=str, nargs='+', help="Data for training. Stored in data folder.", required=True)
    argument_parser.add_argument("--offset-distance", "-d", help="Distance between the sockets in meters, default is 0.05", default=0.05, type=float)
    argument_parser.add_argument("--regularizer", "-reg", help="Regularizer for the optimization", default=1e-4, type=float)
    argument_parser.add_argument("--end-effector", "-ee", help="End effector link", default="ball_link")
    argument_parser.add_argument("--root-link", "-rl", help="Root link", default="base_link")
    argument_parser.add_argument("--variance-noise", "-v", help="Variance of the noise injected to the initial robot parameters", default=0.00, type=float)
    argument_parser.add_argument("--number-samples", "-n", help="Number of samples to use", default=None, type=int)
    argument_parser.add_argument("--latex", "-l", action='store_true', help="Use latex for saving plots")

    args = argument_parser.parse_args()
    
    model = args.model
    train_data = args.data
    robot_name = os.path.dirname(train_data[0])
    variance_noise = args.variance_noise
    end_effector = args.end_effector
    root_link = args.root_link
    offset_distance = args.offset_distance
    regularizer = args.regularizer
    number_samples = args.number_samples
    latex = args.latex
    saving_steps = True

    script_directory = os.path.abspath(__file__)
    parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
    
    output_path = os.path.abspath(os.path.join(parent_directory, 'calibrated_urdf', robot_name + '_comparison'))
    urdf_path = os.path.abspath(os.path.join(parent_directory, 'urdf', model + ".urdf"))

    # Check if paths exist
    check_urdf_path(urdf_path)

    data_path = []
    for d in train_data:
        data_path.append(os.path.abspath(os.path.join(parent_directory, 'data', d)))
    check_data_path(data_path)
    
    # Get all available data folders in the robot directory
    robot_data_dir = os.path.abspath(os.path.join(parent_directory, 'data', robot_name))
    all_data_folders = [os.path.join(robot_data_dir, d) for d in os.listdir(robot_data_dir) 
                       if os.path.isdir(os.path.join(robot_data_dir, d))]

    # Get the test data path (all folders not in training data)
    train_data_full_paths = [os.path.abspath(os.path.join(parent_directory, 'data', d)) for d in train_data]
    test_data_path = [folder for folder in all_data_folders if folder not in train_data_full_paths]

    print(f"Training data folders: {len(train_data_full_paths)}")
    print(f"Test data folders: {len(test_data_path)}")

    # Clean output folder if it exists
    if os.path.exists(output_path):
        shutil.rmtree(output_path)
    os.makedirs(output_path, exist_ok=True)

    # Create configuration
    config = {
        'urdf': model,
        'robot-name': robot_name,
        'variance_noise': variance_noise,
        'end_effector': end_effector,
        'root_link': root_link,
        'saving_steps': saving_steps,
        'number_samples': number_samples,
        'offset_distance': offset_distance,
        'regularizer': regularizer,
        'train_data_path': data_path,
        'test_data_path': test_data_path,
        'study_type': 'ablation_study',
        'optimization_scenarios': [
            'Petrič et al.',
            'Minimize Variance', 
            'Minimize Variance + Distortion Error',
            'Minimize Variance + Distortion Error + Regularization'
        ]
    }
    
    with open(f"{output_path}/config.yaml", "w") as f:
        yaml.dump(config, f)

    print(f"Output path: {output_path}")
    print(f"Training on: {data_path}")
    print(f"Testing on: {test_data_path}")
 
    # Run optimization
    optimizer = ParameterOptimizer(output_path)
    optimizer.set_offset_distance(offset_distance)
    optimizer.set_regulizer_weight(regularizer)
    optimizer.load_model(urdf_path)
    optimizer.read_data(data_path, number_samples=number_samples)   
    optimizer.create_symbolic_fk(root_link, end_effector)
    
    # Load parameter configuration
    with open(f"{parent_directory}/config_optimizer/{model}.yaml", "r") as f:
        parameters = yaml.load(f, Loader=yaml.FullLoader)
    
    optimizer.select_parameters(variance_noise=variance_noise, selected_parameters=parameters['joints'])
    optimizer.data = remove_outliers(optimizer._model, optimizer.data)
    
    print("\nEvaluating initial model...")
    optimizer.evaluate_fks(verbose=True)
    
    # Evaluate initial model on test data for comparison
    print("\nEvaluating initial model on test data...")
    initial_model = optimizer._model
    initial_test_stats = evaluate_model(initial_model, test_data_path, offset_distance=offset_distance)
    
    # Define optimization scenarios for ablation study
    optimization_scenarios = [
        {
            'name': 'Petrič\net al.',
            'params': {'use_dynamic_means': False, 'use_distortion_error': False, 'use_regularization': False}
        },
        {
            'name': 'Variance\nonly',
            'params': {'use_dynamic_means': True, 'use_distortion_error': False, 'use_regularization': False}
        },
        {
            'name': 'Variance +\nDistortion',
            'params': {'use_dynamic_means': True, 'use_distortion_error': True, 'use_regularization': False}
        },
        {
            'name': 'Variance +\nDistortion +\nReg.',
            'params': {'use_dynamic_means': True, 'use_distortion_error': True, 'use_regularization': True}
        }
    ]
    
    # Store results for each scenario
    ablation_results = {}
    
    for i, scenario in enumerate(optimization_scenarios):
        print(f"\n{'='*60}")
        print(f"Running Scenario {i+1}/4: {scenario['name'].replace(chr(10), ' ')}")
        print(f"{'='*60}")
        
        # Create a fresh optimizer for each scenario
        scenario_optimizer = ParameterOptimizer(output_path)
        scenario_optimizer.set_offset_distance(offset_distance)
        scenario_optimizer.set_regulizer_weight(regularizer)
        scenario_optimizer.load_model(urdf_path)
        scenario_optimizer.read_data(data_path, number_samples=number_samples)   
        scenario_optimizer.create_symbolic_fk(root_link, end_effector)
        scenario_optimizer.select_parameters(variance_noise=variance_noise, selected_parameters=parameters['joints'])
        scenario_optimizer.data = remove_outliers(scenario_optimizer._model, scenario_optimizer.data)
        
        # Run optimization with scenario-specific parameters
        print(f"Optimizing with parameters: {scenario['params']}")
        scenario_optimizer.optimize(saving_steps=False, **scenario['params'])
        
        # Evaluate on test data
        print("Evaluating optimized model on test data...")
        final_model = scenario_optimizer._model
        test_stats = evaluate_model(final_model, test_data_path, offset_distance=offset_distance)
        
        # Store results
        ablation_results[scenario['name']] = test_stats
        
        print(f"Scenario {i+1} completed!")
    
    # Create and save ablation study violin plots
    print(f"\n{'='*60}")
    print("Generating ablation study violin plots...")
    print(f"{'='*60}")
    
    fig = plot_ablation_box_plots(ablation_results, initial_test_stats, offset_distance, latex)
    
    # Save plots
    if latex:
        fig.savefig(f"{output_path}/ablation_study_violinplots.pgf", bbox_inches='tight', dpi=300)
        fig.savefig(f"{output_path}/ablation_study_violinplots.pdf", bbox_inches='tight', dpi=300)
    else:
        fig.savefig(f"{output_path}/ablation_study_violinplots.png", dpi=300, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        fig.savefig(f"{output_path}/ablation_study_violinplots.pdf", bbox_inches='tight', dpi=300,
                   facecolor='white', edgecolor='none')
    
    plt.show()
    
    # Print detailed statistics
    print_ablation_statistics(ablation_results, initial_test_stats)
    
    print(f"\nResults saved to: {output_path}")


if __name__ == "__main__":
    main()
