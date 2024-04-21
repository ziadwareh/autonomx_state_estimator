import matplotlib.pyplot as plt
import bagpy
import pandas as pd
import numpy as np
import scipy.fft
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    # read rosbag
    bag = bagpy.bagreader('/home/ziad/ever_comp/src/autonomx_state_estimator/rosbags/kf_infinity_11_2024-04-20-00-12-02.bag')
    method = 'butterworth_trapezoidal'

    # extract topics of interest
    ground_truth_topic_data = pd.read_csv(bag.message_by_topic('/odom'))
    velocity_estimations_topic_data = pd.read_csv(bag.message_by_topic('/current_linear_velocity'))
    heading_estimations_topic_data = pd.read_csv(bag.message_by_topic('/current_heading'))
    # imu_topic_data = pd.read_csv(bag.message_by_topic('/Imu'))
    # filtered_imu_topic_data = pd.read_csv(bag.message_by_topic('/Imu_filtered'))

    '''
        Plotting Ground Truth Velocity Compared to Estimated Velocities
    '''

    # extract the data i need as pd.sereis and store them in a list
    ground_truth = [ground_truth_topic_data['twist.twist.linear.x'], ground_truth_topic_data['twist.twist.linear.y'], ground_truth_topic_data['twist.twist.linear.z']]
    estimates = [velocity_estimations_topic_data['x'], velocity_estimations_topic_data['y'], velocity_estimations_topic_data['z']]

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
    Plotting the ground truth heading compared to the estimated heading
    '''

    ground_truth_orientation = [ground_truth_topic_data['pose.pose.orientation.x'], ground_truth_topic_data['pose.pose.orientation.y'],ground_truth_topic_data['pose.pose.orientation.z'], ground_truth_topic_data['pose.pose.orientation.w']]
    estimated_heading = heading_estimations_topic_data['data']

    ground_truth_heading = np.zeros_like(ground_truth_orientation[1].values)

    # Converting the orientation from quaternion to euler and extracting the yaw

    for i in range(len(ground_truth_heading)):
        ground_truth_heading[i] = euler_from_quaternion((ground_truth_orientation[0][i], ground_truth_orientation[1][i], ground_truth_orientation[2][i], ground_truth_orientation[3][i]))[2]

    fig2, axes2 = plt.subplots(1, 1, figsize=(15, 10))
    axes2.plot(range(len(ground_truth_heading)), ground_truth_heading, label='Ground Truth', color='red', linestyle='--')
    axes2.plot(range(len(estimated_heading)), estimated_heading, label='Estimate', color='black')
    axes2.set_ylabel('Heading')
    axes2.legend()
    axes2.grid()

    plt.xlabel("Time Step")
    plt.tight_layout()  # Adjust spacing between subplots (optional)
    # plt.show()

    '''
        Plotting the IMU data
    '''

    # Extract the data i need
    # imu_accelrations = [imu_topic_data['linear_acceleration.x'], imu_topic_data['linear_acceleration.y'], imu_topic_data['linear_acceleration.z']]
    # imu_accelrations_filtered = [filtered_imu_topic_data['x'], filtered_imu_topic_data['y'], filtered_imu_topic_data['z']- np.ones_like(filtered_imu_topic_data['z'])*9.81]

    # fig2, axes2 = plt.subplots(3, 1, figsize=(15, 10))
    # for i, ax in enumerate(axes2):
    #     ax.plot(range(len(imu_accelrations[i])), imu_accelrations[i], label='Imu Raw Data', color='red')
    #     ax.plot(range(len(imu_accelrations_filtered[i])), imu_accelrations_filtered[i], label='Imu Filtered Data', color='black')

    #     axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
    #     ax.set_ylabel(f'{axis} linear acceleration')
    #     ax.legend()
    #     ax.grid()
    

    # plt.xlabel("Time Step")
    # plt.tight_layout()  # Adjust spacing between subplots (optional)

    '''
        Performing Fourier Analysis to decompose the signal to determine the Noise frequency
        
    '''
    # # I need to determine my sampling rate
    # df = bag.topic_table
    # fs = df[df['Topics'] == '/Imu']['Frequency']
    # T = 1/fs

    # # plotting my fourier transform
    # fig3, axes3 = plt.subplots(3, 1, figsize=(15, 10))
    # for i, ax in enumerate(axes3):
    #     frequency_axis = np.linspace(-fs/2,fs/2,len(np.array(imu_accelrations[i])))
    #     fourier_transform = scipy.fft.fft(np.array(imu_accelrations[i]))
    #     ax.plot(frequency_axis, fourier_transform, color='black')
    #     axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
    #     ax.set_ylabel(f'{axis} linear acceleration')
    #     ax.grid()
    
    # plt.xlabel("Frequency (Hz)")
    # plt.tight_layout()  # Adjust spacing between subplots (optional)
    
    '''
        Implementing the butterworth second order low pass filter:

        y(n) = A y(n-1) + B y(n-2) + C x(n) + D x(n-1) + E x(n-2)

        A = -b/c
        B = -a/c
        C = (wa*wa)/c
        D = (2*wa*wa)/c
        E = (wa*wa)/c

        a = k^2 - (wa*k*sqrt(2)) + wa^2
        b = -2*k^2 + 2*wa^2
        c = k^2 + (wa*k*sqrt(2)) + wa^2

        wa is the prewarped frequency and it is equal to:
        wa =k*tan(w_d/2)

        with wd being the normalized desired cut_off frequency:
        w_d = 2*pi*fc/fs

        with fc being the cut_off frequency and fs being the smapling frequency

        finally K is the biliniar transform gain, as well as the prewarping gain, therefore it should be 2/T
    '''
    # fc = 2
    # k = 1/T
    # # prewarping
    # w_d = 2*np.pi*fc/fs
    # wa =k*np.tan(w_d/2)
    # a = (k*k) - (wa*k*np.sqrt(2)) + (wa*wa)
    # b = -2*(k*k) + (2*wa*wa)
    # c = (k*k) + (wa*k*np.sqrt(2)) + (wa*wa)

    # A = -b/c
    # B = -a/c
    # C = (wa*wa)/c
    # D = (2*wa*wa)/c
    # E = (wa*wa)/c

    # filtered_signal = np.zeros_like(np.array(imu_accelrations))

    # for i in range(3):
    #     output = np.zeros(2)
    #     input = np.zeros(2)
    #     current_imu_signal = np.array(imu_accelrations[i])
    #     for j in range(len(imu_accelrations[i])):
    #         filtered_signal[(i,j)] = A*output[1] + B*output[0] + C*current_imu_signal[j] + D*input[1] + E*input[0]

    #         output[0] = output[1]
    #         output[1] = filtered_signal[i][j]

    #         input[0] = input[1]
    #         input[1] = current_imu_signal[j]
    
    
    '''
        Plotting the filtered data against the raw signal as well as conducting the fourier signal decomposition
    '''
    # fig4, axes4 = plt.subplots(3, 1, figsize=(15, 10))
    # for i, ax in enumerate(axes4):
    #     ax.plot(range(len(imu_accelrations[i])), imu_accelrations[i], label='Imu Data', color='red')
    #     ax.plot(range(len(filtered_signal[i])), filtered_signal[i], label='Imu Data', color='black')
    #     axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
    #     ax.set_ylabel(f'{axis} imu readings')
    #     ax.legend(['unfiltered data','filtered data'])
    #     ax.grid()
    
    # plt.xlabel("Time step")
    # plt.tight_layout()  # Adjust spacing between subplots (optional)


    # fig5, axes5 = plt.subplots(3, 1, figsize=(15, 10))
    # for i, ax in enumerate(axes5):
    #     frequency_axis = np.linspace(-fs/2,fs/2,len(filtered_signal[i]))
    #     fourier_transform = scipy.fft.fft(filtered_signal[i])
    #     ax.plot(frequency_axis, fourier_transform, color='black')
    #     axis = 'X' if i==0 else ('Y' if i==1 else 'Z')
    #     ax.set_ylabel(f'{axis} linear acceleration')
    #     ax.grid()
    
    # plt.xlabel("Frequency (Hz)")
    # plt.tight_layout()  # Adjust spacing between subplots (optional)

    
    ''' 
        save plots
    '''
    # fig1.savefig(f'/home/ziad/ever_comp/src/autonomx_state_estimator/Plots/filtering/velocity_estimates_{method}.png')
    # fig2.savefig(f'/home/ziad/ever_comp/src/autonomx_state_estimator/Plots/filtering/imu_linear_acceleration_raw_data_{method}.png')
    # fig3.savefig(f'/home/ziad/ever_comp/src/autonomx_state_estimator/Plots/filtering/fourier_{method}.png')
    # fig4.savefig(f'/home/ziad/ever_comp/src/autonomx_state_estimator/Plots/filtering/imu_linear_acceleration_filtered_{method}.png')
    # fig5.savefig(f'/home/ziad/ever_comp/src/autonomx_state_estimator/Plots/filtering/fourier_filtered_{method}.png')
    plt.show()