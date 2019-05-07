#!/usr/bin/env python3
# yapf: disable

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def trendPlot(dataframe, x_key, y_key, ax, color, label, logx=True, logy=False,
              degree=5):
    trend_fn = np.poly1d(
        np.polyfit(x=np.log(dataframe[x_key].tolist()),
                   y=dataframe[y_key].tolist(),
                   deg=degree))
    dataframe.plot(x=x_key, y=y_key, ax=ax, logx=logx, logy=logy,
                   kind='scatter', color=color, label=label)
    plt.plot(dataframe[x_key].tolist(),
             trend_fn(np.log(dataframe[x_key].tolist())),
             color=color)

#################
# Load the data #
#################
rosbag_duration = 272
number_solver_calls = 25

plot_performance = True
plot_solver_summary = False

mapper_test_bench_log_dir = \
    '/home/victor/data/voxgraph/mapper_test_bench_stats/'
mapper_test_bench_filename = 'combined_implicit_explicit'

mapper_test_bench_log_filepath = os.path.join(
    mapper_test_bench_log_dir, mapper_test_bench_filename + '.csv')
results_dataframe = pd.read_csv(mapper_test_bench_log_filepath, skiprows=[0, 1])

# Strip leading/trailing white spaces on column names
results_dataframe.columns = results_dataframe.columns.str.strip()

# Compute the pose errors
for pose_key in ['average_submap_pose_error', 'last_submap_pose_error',
                 'used_rigid_body_alignment']:
    # Parse the poses into vectors
    results_dataframe[pose_key] = results_dataframe[pose_key].apply(
        lambda x: x.split('; '))
    # Compute the distance errors
    results_dataframe[pose_key+"_dist"] = results_dataframe[pose_key].apply(
        lambda x: np.sqrt(float(x[0])**2+float(x[1])**2+float(x[2])**2))

# Normalize time over data length
results_dataframe['normalized_runtime'] =\
    results_dataframe['total_milliseconds'].apply(
        lambda x: x/(rosbag_duration * 1000))

# Compute the total solver time
results_dataframe['normalized_total_solver_time'] =\
    results_dataframe['average_solver_time'].apply(
        lambda x: x * number_solver_calls / (rosbag_duration))

##################
# Generate plots #
##################
sampled_results_dataframe = results_dataframe[
    results_dataframe['sampling_ratio'] > 0.0001]

for registration_method in ['explicit_to_implicit', 'implicit_to_implicit']:
    figure_path = os.path.join(mapper_test_bench_log_dir,
                               mapper_test_bench_filename
                               + '-' + registration_method)

    filtered_dataframe = sampled_results_dataframe[
        sampled_results_dataframe['registration_method'] == registration_method]

    # Performance in function of sampling ratio
    if plot_performance:
        fig = plt.figure()

        # Sampling ratio vs runtime
        ax1 = plt.subplot(3, 1, 1)
        trendPlot(filtered_dataframe, 'sampling_ratio', 'normalized_runtime',
                  ax1, 'blue', label='Total')
        trendPlot(filtered_dataframe, 'sampling_ratio',
                  'normalized_total_solver_time', ax1, 'green', label='Solver')
        ax1.set_ylim(-0.02, 1.0)
        ax1.set_ylabel('Normalized runtime')
        ax1.legend(loc='upper right')

        # Sampling ratio vs RMSE
        ax2 = plt.subplot(3, 1, 2, sharex=ax1)
        trendPlot(filtered_dataframe, 'sampling_ratio', 'max_error', ax2, 'red',
                  label='Max error', degree=3)
        ax2.set_ylim(0.0, 2.5)
        ax2.set_ylabel('Max error')
        ax2.legend(loc='upper left')
        ax3 = ax2.twinx()
        trendPlot(filtered_dataframe, 'sampling_ratio', 'rmse', ax3, 'blue',
                  label='RMSE', degree=3)
        ax3.set_ylim(0.0, 0.4)
        ax3.set_yticks(np.arange(0.0, 0.5, 0.1))
        ax3.set_ylabel('RMSE')
        ax3.legend(loc='lower right')


        # Sampling ratio vs ATE
        ax5 = plt.subplot(3, 1, 3, sharex=ax1)
        trendPlot(filtered_dataframe, 'sampling_ratio',
                  'last_submap_pose_error_dist', ax5, 'red', label='Final',
                  degree=3)
        trendPlot(filtered_dataframe, 'sampling_ratio',
                  'average_submap_pose_error_dist', ax5, 'blue',
                  label='Average', degree=3)
        ax5.set_ylim(-0.1, 6.0)
        ax5.set_ylabel('Pose error')
        ax5.legend(loc='upper left')

        # Finish the plot
        ax5.set_xlabel('Sampling ratio')
        ax1.set_xlim(1.1, -0.1)
        fig.suptitle('Performance in function of sampling ratio')
        plt.figtext(.50, .92, 'Using ' + registration_method.replace('_', ' ')
                    + ' constraints', horizontalalignment='center')
        plt.savefig(figure_path + '-performance', format='svg')

    # Solver statistics in function of sampling ratio
    if plot_solver_summary:
        fig = plt.figure()

        # Number of residuals
        ax1 = plt.subplot(3, 1, 1)
        trendPlot(filtered_dataframe, 'sampling_ratio',
                  'max_num_residuals', ax1, 'red', label='Max')
        trendPlot(filtered_dataframe, 'sampling_ratio', 'average_num_residuals',
                  ax1, 'blue', label='Average')
        ax1.set_ylim(-0.1e7, 1.5e7)
        ax1.set_ylabel('Residual count')
        ax1.legend(loc='upper right')

        # Number of iterations
        ax3 = plt.subplot(3, 1, 2, sharex=ax1)
        trendPlot(filtered_dataframe, 'sampling_ratio', 'max_solver_iterations',
                  ax3, 'red', label='Max', degree=1)
        trendPlot(filtered_dataframe, 'sampling_ratio',
                  'average_solver_iterations', ax3, 'blue', label='Average',
                  degree=1)
        ax3.set_ylim(0.0, 40)
        ax3.set_ylabel('Iteration count')
        ax3.legend(loc='upper right')

        # Solve time
        ax5 = plt.subplot(3, 1, 3, sharex=ax1)
        trendPlot(filtered_dataframe, 'sampling_ratio', 'max_solver_time', ax5,
                  'red', label='Max', degree=5)
        trendPlot(filtered_dataframe, 'sampling_ratio', 'average_solver_time',
                  ax5, 'blue', label='Average', degree=3)
        ax5.set_ylim(-3.0, 40.0)
        ax5.set_ylabel('Solver time')
        ax5.legend(loc='upper right')

        # Finish the plot
        ax5.set_xlabel('Sampling ratio')
        ax1.set_xlim(1.1, -0.1)
        fig.suptitle("Solver statistics in function of sampling ratio")
        plt.figtext(.50, .92, 'Using ' + registration_method.replace('_', ' ')
                    + ' constraints', horizontalalignment='center')
        plt.savefig(figure_path + '-solver_statistics', format='svg')

plt.show()
