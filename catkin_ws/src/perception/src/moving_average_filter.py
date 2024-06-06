#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from collections import deque
from enum import Enum


class FilteringMode(Enum):
    NORMAL = 1
    WEIGHTED = 2


class MovingAverageFilter:
    def __init__(self, sample_size: int, mode: FilteringMode):
        self.sample_size = sample_size
        self.mode = mode
        self.samples = deque(maxlen=sample_size)
        self.weights = list(range(1, sample_size + 1))
        self.filtering_result = None

    def add_sample(self, new_sample: float):
        self.samples.append(new_sample)
        self.filtering_result = self.update(len(self.samples))

    def update(self, sample_size: float) -> float:
        sum_values = 0
        denominator = 0

        if self.mode == FilteringMode.NORMAL:
            sum_values = sum(self.samples)
            denominator = sample_size
        elif self.mode == FilteringMode.WEIGHTED:
            for i in range(sample_size):
                sum_values += self.samples[i] * self.weights[i]
                denominator += self.weights[i]

        assert denominator != 0
        return sum_values / denominator

if __name__ == "__main__":
    # Example usage
    filter_normal = MovingAverageFilter(5, FilteringMode.NORMAL)
    filter_weighted = MovingAverageFilter(5, FilteringMode.WEIGHTED)

    samples = [10, 20, 30, 40, 50]
    for sample in samples:
        filter_normal.add_sample(sample)
        filter_weighted.add_sample(sample)
        print(f"Normal Mode Result: {filter_normal.filtering_result}")
        print(f"Weighted Mode Result: {filter_weighted.filtering_result}")

    # Adding more samples to test the moving window behavior
    additional_samples = [60, 70]
    for sample in additional_samples:
        filter_normal.add_sample(sample)
        filter_weighted.add_sample(sample)
        print(f"Normal Mode Result: {filter_normal.filtering_result}")
        print(f"Weighted Mode Result: {filter_weighted.filtering_result}")
