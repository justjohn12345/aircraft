// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Test-only access to the generated model's private state and parameters (as in the stage checkers). Include this
// before anything else that includes FmgcComputer.h, so that the class is first seen with its members public.

#pragma once

#define private public
#include "FmgcComputer.h"
#undef private
