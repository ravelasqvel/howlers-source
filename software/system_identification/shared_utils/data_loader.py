"""
Data loading utilities for experimental robot data.

This module provides functions to download and load experimental data files
from remote sources and parse them into usable formats.
"""

import requests
import pandas as pd
from typing import List


def download_files(urls: List[str], filenames: List[str]) -> None:
    """
    Download files from a list of URLs and save them with specified filenames.

    Args:
        urls: List of URLs to download from.
        filenames: List of filenames to save the downloaded files as.
                  Must be the same length as the urls list.

    Raises:
        ValueError: If the number of URLs and filenames don't match.
    """
    if len(urls) != len(filenames):
        print("Error: The number of URLs and filenames must be the same.")
        return

    for url, filename in zip(urls, filenames):
        try:
            print(f"Downloading from: {url}")
            r = requests.get(url, stream=True)
            r.raise_for_status()

            with open(filename, 'wb') as f:
                for chunk in r.iter_content(chunk_size=8192):
                    f.write(chunk)

            print(f"File downloaded successfully: {filename}")
        except requests.exceptions.RequestException as e:
            print(f"Error downloading {url}: {e}")
        except Exception as e:
            print(f"An error occurred while processing {url}: {e}")


def load_experiment_file(filename: str) -> pd.DataFrame:
    """
    Load experimental data file into a pandas DataFrame.

    Expected columns: t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp

    Args:
        filename: Path to the experimental data file (CSV format).

    Returns:
        DataFrame with experimental data and proper column names.
    """
    expected_columns = [
        "t", "ax", "ay", "alpha",
        "w1", "w2", "w3",
        "u1", "u2", "u3",
        "vbx_sp", "vby_sp"
    ]

    df = pd.read_csv(
        filename,
        sep=r",\s*",
        engine="python",
        names=expected_columns,
        skiprows=1
    )

    return df
