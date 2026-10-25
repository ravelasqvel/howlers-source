/*
 ============================================================================
 Name        : main.c
 Author      : MaverickST
 Version     : 2.0
 Copyright   : MIT
 Description : State Estimation Examples - Interactive Menu System

 This is a unified menu system that allows you to run different filter
 examples from a single executable.

 Powershell Build Commands:
    Set-Location 'e:\Control and State Estimation'

    # Build unified menu (all examples in one executable):
    .\build.ps1 menu
    .\menu.exe

    # Or build examples separately:
    .\build.ps1 ukf    # Creates ukf_demo.exe
    .\build.ps1 pf     # Creates pf_demo.exe
    .\build.ps1 all    # Builds all separate executables
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>

/* Forward declarations - these functions are defined in Examples/ */
extern int run_ukf_example(void);
extern int run_pf_example(void);
extern int run_ekf_example(void);

void print_banner(void)
{
    printf("\n");
    printf("============================================\n");
    printf("   State Estimation Examples\n");
    printf("============================================\n");
    printf("\n");
    printf("Available Filters:\n");
    printf("  1. UKF - Unscented Kalman Filter\n");
    printf("  2. PF  - Particle Filter\n");
    printf("  3. EKF - Extended Kalman Filter\n");
    printf("\n");
    printf("============================================\n");
    printf("\n");
}

void show_menu(void)
{
    printf("Select an example to run:\n");
    printf("  [1] UKF - Constant Velocity 2D Tracking\n");
    printf("  [2] PF  - Robot Localization with Landmarks\n");
    printf("  [3] EKF - Constant Velocity 2D Tracking\n");
    printf("  [0] Exit\n");
    printf("\n");
    printf("Enter your choice: ");
}

int main(void)
{
    int choice;
    int running = 1;

    print_banner();

    while (running)
    {
        show_menu();

        if (scanf("%d", &choice) != 1)
        {
            /* Clear invalid input */
            while (getchar() != '\n')
                ;
            printf("\nInvalid input! Please enter a number.\n\n");
            continue;
        }

        /* Clear input buffer */
        while (getchar() != '\n')
            ;

        printf("\n");

        switch (choice)
        {
        case 1:
            printf("============================================\n");
            printf("Running UKF Example\n");
            printf("============================================\n\n");
            run_ukf_example();
            break;

        case 2:
            printf("============================================\n");
            printf("Running Particle Filter Example\n");
            printf("============================================\n\n");
            run_pf_example();
            break;

        case 3:
            printf("============================================\n");
            printf("Running Extended Kalman Filter Example\n");
            printf("============================================\n\n");
            run_ekf_example();
            break;

        case 0:
            printf("Exiting...\n\n");
            running = 0;
            break;

        default:
            printf("Invalid choice! Please select 0-3.\n\n");
            break;
        }

        if (running && choice >= 1 && choice <= 3)
        {
            printf("\nPress Enter to return to menu...");
            getchar();
            printf("\n");
        }
    }

    printf("============================================\n");
    printf("Thank you for using State Estimation Examples!\n");
    printf("============================================\n\n");

    return 0;
}
