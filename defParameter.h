// Parameters for KITTI dataset

// The number of superpixel
int superpixelTotal = 400;

// The number of iterations
int outerIterationTotal = 5;
int innerIterationTotal = 10;

// Weight parameters
double lambda_pos = 1000.0;
double lambda_depth = 2000.0;
double lambda_bou = 1000.0;
double lambda_smo = 300.0;

// Inlier threshold
double lambda_d = 3.0;

// Penalty values
double lambda_hinge = 1.0;
double lambda_occ = 2.0;
double lambda_pen = 30.0;
