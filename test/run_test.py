#!/usr/bin/env python

import unittest
import rosunit
import rostest


if __name__ == "__main__":
    rosunit.unitrun('configuration_node','run_test','test_cases.RunTests')
