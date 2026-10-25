idf_component_register(SRCS 
                            # Core matrix operations (USED)
                            "chol.c"          # Cholesky decomposition - used by UKF
                            "dot.c"           # Dot product - used by chol.c
                            "inv.c"           # Matrix inversion - used by EKF and UKF
                            "lup.c"           # LU decomposition with pivoting - used by inv.c
                            "mul.c"           # Matrix multiplication - used by EKF and UKF
                            "tran.c"          # Matrix transpose - used by EKF and inv.c
                            "angle_utils.c"   # Angle utilities - used by all filters
                            
                            # UNUSED - commented out to reduce binary size
                            # "cholupdate.c"
                            # "conv.c"
                            # "conv2.c"
                            # "conv2fft.c"
                            # "convfft.c"  
                            # "det.c"
                            # "eye.c"
                            # "linsolve_chol.c"
                            # "linsolve_gauss.c"
                            # "linsolve_lower_triangular.c"
                            # "linsolve_lup.c"
                            # "linsolve_qr.c"
                            # "linsolve_upper_triangular.c"
                            # "norm.c"
                            # "pinv.c"
                            # "qr.c"
                            # "rank.c"
                            # "svd.c"
                    INCLUDE_DIRS ".")
