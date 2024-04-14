package org.team100.lib.visualization;

/**
 * Visualizations all use the same pattern: they asynchonously observe
 * something, and reflect the observations in network tables, where they can be
 * rendered in glass.
 * 
 * There are no public methods or constructors, just a static method that
 * creates the visualization and renders it with the async runner.
 * 
 * (... except the trajectory visualizer is not an asynchronous observer.)
 */