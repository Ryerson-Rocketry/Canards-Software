// file for baro-imu ekf for altitude and velocity determination
// https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/pdf/sensors-14-13324.pdf 

// there will be a change to the paper, since we already have attitude orientation quaternian
// we will be skipping the attitude quaternian ekf in the paper
// the attitude part will be sent to the EKF via queue or publish and scribe type shi (idk)
// 