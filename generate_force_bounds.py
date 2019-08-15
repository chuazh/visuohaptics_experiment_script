#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pl

def loadfile(filename):
    data = np.loadtxt(filename, delimiter=',')
    data = data[1:-1, :]
    return data

def split_data(data, displacement_level):
    # returns an array for that target displacement level
    # input:  data array Nx16
    #         target displacement to extract from data array
    # output: list of data arrays for that discplament level. The length of the list is the number of trials at that target displacement.  
    
    idx = np.where(data[:, 0] == displacement_level) # find the entries in the data that equal the target displacement
    split_disp = data[idx] # grab only those entries
    max_trials = np.amax(split_disp[:,1]) # look at the trial number to get the max trial number
    idxes =[]
    output=[]
    for i in range(0,int(max_trials)): # for each trial do
        idxes.append(np.where(split_disp[:, 1] == i+1)) # append to list an array of idxes corresponding to that trial
        output.append(split_disp[idxes[i]]) # append to list an array of the outputs of that trial.
        output[i][:,6] = output[i][:,6]-output[i][0,6] # zero out column 6 using the initial displacement

    return output

def interpAndAvg(data):
    # input:    data as a list of data arrays corresponding to the same forces.
    # output:   an vector of the displacement and a vector of force
    # This function first splits each array (in the list) into an elongation and retraction portion and interpolates the curves using the first curves displacement values
    # Once they are interpolated into the same x values, it averages them to get a single curve.

    numData = len(data)
    new_data = []

    # split the array into elongation and relaxation portions
    for i in range(0, numData):
        idx_max = np.argmax(data[i][:, 13])
        new_data.append(data[i][0:idx_max, :])
        new_data.append(data[i][idx_max:-1, :])

    numData = len(new_data)
    scrubbed_data = []
    interped_forces = []
    interped_displacements = []

    for i in range(0, numData):
        # first determine that the reference array has unique values of force and time
        scrubbed_data.append(check_unique(new_data[i]))

        if i > 0:
            interped_forces.append(np.interp(scrubbed_data[0][:, 6], scrubbed_data[i][:, 6], scrubbed_data[i][:, 13]))
            interped_displacements.append(scrubbed_data[i][:, 6])
            mean_force_arr = np.vstack((mean_force_arr, np.array(interped_forces[i])))
            # mean_disp_arr = np.vstack((mean_disp_arr, np.array(interped_displacements[i])))
        else:
            interped_forces.append(scrubbed_data[i][:, 13]) # initialize the force data points
            interped_displacements.append(scrubbed_data[i][:, 6]) # initialize the displacement data points
            mean_force_arr = np.array(interped_forces[i])
            # mean_disp_arr = np.array(interped_displacements[i])

    mean_force = np.mean(mean_force_arr, 0)
    mean_disp = scrubbed_data[0][:, 6]
    # mean_disp = np.mean(mean_disp_arr,0)

    return mean_disp, mean_force


def check_unique(data):
    # takes in an array
    val, idxF = np.unique(data[:, 13], return_index=True)
    scrubbed_data = data[idxF]
    val, idxD = np.unique(scrubbed_data[:, 6], return_index=True)
    unique_data = scrubbed_data[idxD]

    return unique_data

def fit_poly3(displacement_list, force_list):
    # takes in a list of forces and their corresponding displacements
    # returns a 1-D polynomial class

    numForces = len(force_list)

    # iterate through the force and disp list and populate a 2 arrays of data point pairs
    for i in range(0, numForces):
        if i == 0:
            aggregate_forces = force_list[i]
            aggregate_disp = displacement_list[i]
        else:
            aggregate_forces = np.hstack((aggregate_forces, force_list[i]))
            aggregate_disp = np.hstack((aggregate_disp, displacement_list[i]))

    coeffs = np.polyfit(-aggregate_disp, aggregate_forces, 3)
    p = np.poly1d(coeffs)

    return p


def get_force_error_bounds(polyfunc, target_forces, eps):
    # takes in 1-D polynomial object and a list of target forces from which to calculate an error bounds based on
    # eps value which is given in terms of displacement.

    force_bound = np.zeros((3,len(target_forces)))

    for i in range(0,len(target_forces)):
        roots = (polyfunc - target_forces[i]).r
        rv = roots.real[abs(roots.imag) < 1e-5]
        force_bound[0,i] = target_forces[i]
        force_bound[1,i] = np.absolute(polyfunc(rv + eps))
        force_bound[2,i] = np.absolute(polyfunc(rv-eps))
        print('target force = ' + str(target_forces[i]))
        print('upper bound = ' + str(np.absolute(force_bound[1,i])))
        print('lower bound = ' + str(np.absolute(force_bound[2,i])))
    
    return force_bound

def plot_avgs(displacement_list, force_list):
    
    numForces = len(force_list)
    
    plt.figure()

    for j in range(0, numForces):
        force = force_list[j]
        displacement = displacement_list[j]
        plt.plot(-displacement, force,'b')


def plot_poly3(polyfunc, displacement_list, force_list):
    # taks in 1-D polynomial object and the list of average displacments and forces

    numForces = len(force_list)

    # iterate through the force and disp list and populate a 2 arrays of data point pairs
    for i in range(0, numForces):
        if i == 0:
            aggregate_forces = force_list[i]
            aggregate_disp = displacement_list[i]
        else:
            aggregate_forces = np.hstack((aggregate_forces, force_list[i]))
            aggregate_disp = np.hstack((aggregate_disp, displacement_list[i]))

    x_poly = -displacement_list[-1]
    
    plt.plot(x_poly, polyfunc(x_poly), 'r',linewidth = 4.0)
    plt.plot(-aggregate_disp, aggregate_forces, '.b')
    
def plot_raw(segmented_data):
    list_size = len(segmented_data)

    plt.figure()

    for i in range(0, list_size):
        numTrials = len(segmented_data[i])

        for j in range(0, numTrials):
            force = segmented_data[i][j][:, 13]
            displacement = -segmented_data[i][j][:, 6]
            plt.plot(displacement, force,'b')

    plt.show()
    

#%%
        
if __name__ == "__main__":
    # START MAIN SCRIPT

    p = []

    k= 0 
    
    filename = "071219_5_2" + ".csv"
    
    print(filename)
    data = loadfile(filename)
    target_displacements = np.unique(data[:, 0])
    segmented_data = []
    average_disp_curves = []
    average_force_curves = []
    
    # split the data and average the load and unload portions over all trials at that discplacement level to get a single curve
    for i in range(0, np.shape(target_displacements)[0]):
        segmented_data.append(split_data(data, target_displacements[i]))
        average_disp, average_force = interpAndAvg(segmented_data[i])
        average_disp_curves.append(average_disp)
        average_force_curves.append(average_force)
    
    p.append(fit_poly3(average_disp_curves, average_force_curves))
    
    #plot_avgs(average_disp_curves, average_force_curves)
    
    plot_raw(segmented_data)
    plot_poly3(p[k], average_disp_curves, average_force_curves)
    plt.show()
    
    # curves we like is what will get 
    curves_we_like = [0] # always -1 of the sample number
    f_curves_we_like = []
    d_curves_we_like = []

    ref_curve_idx = 0
    
    # the x offset makes an assumption. that it is not the force sensor that has a offset error, but the length of the sample at zero that has some offset error?
    # need to think more about this...
    
    for i in curves_we_like:
        # the x offset is the difference of the x intercepts of the reference curve and the curve i
        # we get this by taking the largest root that is not complex for both curves.
        x_offset = np.amax(np.roots(p[ref_curve_idx])[np.iscomplex(np.roots(p[ref_curve_idx]))==False])-np.amax(np.roots(p[i])[np.iscomplex(np.roots(p[i]))==False])
        # using the offset and the displacement values of the last curve (why though?) we can recompute the forces but now shifted.
        f_curves_we_like.append(p[i](-(average_disp_curves[-1]+x_offset)))
        d_curves_we_like.append(average_disp_curves[-1])

    p_avg = fit_poly3(d_curves_we_like,f_curves_we_like)
    
    #force_bounds = get_force_error_bounds(p_avg,[1.5,2.5,3.5,4.5,6],2)
    force_bounds = get_force_error_bounds(p_avg,[1,2],2)
    np.savetxt('force_bounds.csv',force_bounds,'%.3f',delimiter=',')
    coeffs = p_avg.coeffs
    coeffs = coeffs.real
    np.savetxt('avg_curves_coeffs.csv',coeffs,'%.3f',delimiter=',')

    