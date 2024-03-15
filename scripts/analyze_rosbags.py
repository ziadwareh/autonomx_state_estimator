import matplotlib.pyplot as plt
import bagpy
import pandas as pd
import numpy as np
import scipy.fft

if __name__ == '__main__':
    # read rosbag
    bag = bagpy.bagreader('/home/ziad/ever_comp/src/state_estimator/rosbags/stationary_unfiltered_2024-03-15-08-38-40.bag')
    method = 'stationary_unfiltered'

    # extract topics of interest
    ground_truth_topic_data = pd.read_csv(bag.message_by_topic('/odom'))
    estimations_topic_data = pd.read_csv(bag.message_by_topic('/current_linear_velocity'))
    imu_topic_data = pd.read_csv(bag.message_by_topic('/Imu'))

    '''
        Plotting Ground Truth Velocity Compared to Estimated Velocities
    '''

    # extract the data i need as pd.sereis and store them in a list
    ground_truth = [ground_truth_topic_data['twist.twist.linear.x'], ground_truth_topic_data['twist.twist.linear.y'], ground_truth_topic_data['twist.twist.linear.z']]
    estimates = [estimations_topic_data['x'], estimations_topic_data['y'], estimations_topic_data['z']]

    fig1, axes1 = plt.subplots(3, 1, figsize=(15, 10))  # Create 3 rows, 1 column subplot
    for i, ax in enumerate(axes1):
        ax.plot(range(len(ground_truth[i])), ground_truth[i], label='Ground Truth', color='red', linestyle = '--')
        ax.plot(range(len(estimates[i])), estimates[i], label='Estimate', color='black')
        axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
        ax.set_ylabel(f'{axis} linear velocity')
        ax.legend()
        ax.grid()

    plt.xlabel("Time Step")
    plt.tight_layout()  # Adjust spacing between subplots (optional)

    '''
        Plotting the IMU raw data
    '''

    # Extract the data i need
    imu_accelrations = [imu_topic_data['linear_acceleration.x'], imu_topic_data['linear_acceleration.y'], imu_topic_data['linear_acceleration.z']]
    
    fig2, axes2 = plt.subplots(3, 1, figsize=(15, 10))
    for i, ax in enumerate(axes2):
        ax.plot(range(len(imu_accelrations[i])), imu_accelrations[i], label='Imu Data', color='black')
        axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
        ax.set_ylabel(f'{axis} linear acceleration')
        ax.legend()
        ax.grid()
    

    plt.xlabel("Time Step")
    plt.tight_layout()  # Adjust spacing between subplots (optional)

    '''
        Performing Fourier Analysis to decompose the signal to determine the Noise frequency
        
    '''
    # I need to determine my sampling rate
    df = bag.topic_table
    fs = df[df['Topics'] == '/Imu']['Frequency']

    fig3, axes3 = plt.subplots(3, 1, figsize=(15, 10))
    for i, ax in enumerate(axes3):
        frequency_axis = np.linspace(-fs/2,fs/2,len(np.array(imu_accelrations[i])))
        fourier_transform = scipy.fft.fft(np.array(imu_accelrations[i]))
        ax.plot(frequency_axis, fourier_transform, color='black')
        axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
        ax.set_ylabel(f'{axis} linear acceleration')
        ax.grid()
    
    plt.xlabel("Frequency (Hz)")
    plt.tight_layout()  # Adjust spacing between subplots (optional)
    
    
    # save plots
    fig1.savefig(f'/home/ziad/ever_comp/src/state_estimator/Plots/filtering/velocity_estimates_{method}.png')
    fig2.savefig(f'/home/ziad/ever_comp/src/state_estimator/Plots/filtering/imu_linear_acceleration_raw_data_{method}.png')
    fig3.savefig(f'/home/ziad/ever_comp/src/state_estimator/Plots/filtering/fourier_{method}.png')
    plt.show()
