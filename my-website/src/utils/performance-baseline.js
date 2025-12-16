/**
 * Performance Baseline Measurement for Search Feature
 *
 * This script measures the performance characteristics of the search functionality
 * to ensure it meets the specified requirements:
 * - <500ms search update after typing stops
 * - <10% performance degradation compared to baseline
 * - <200ms p95 search update time
 */

// Import search functions for testing
import { searchContent, indexContent } from './searchService';

/**
 * Measures baseline performance without search functionality
 */
function measureBaselinePerformance() {
  const start = performance.now();

  // Simulate baseline operations (DOM access, etc.)
  const operations = 1000;
  for (let i = 0; i < operations; i++) {
    // Simple operations that mimic baseline page behavior
    const test = i * 2 + 1;
  }

  const end = performance.now();
  return end - start;
}

/**
 * Measures search performance with various query types
 */
function measureSearchPerformance() {
  const queries = [
    'robotics',
    'algorithm',
    'machine learning',
    'AI',
    'neural network',
    'path planning',
    'computer vision',
    'control system'
  ];

  const executionTimes = [];

  for (const query of queries) {
    const start = performance.now();

    // Execute search with default options
    const results = searchContent(query, {
      partialMatch: true,
      maxResults: 10
    });

    const end = performance.now();
    const duration = end - start;
    executionTimes.push(duration);
  }

  return executionTimes;
}

/**
 * Calculates performance statistics
 */
function calculateStats(executionTimes) {
  const sortedTimes = [...executionTimes].sort((a, b) => a - b);
  const total = sortedTimes.reduce((sum, time) => sum + time, 0);
  const avg = total / sortedTimes.length;

  // Calculate p95 (95th percentile)
  const p95Index = Math.floor(0.95 * sortedTimes.length) - 1;
  const p95 = sortedTimes[p95Index];

  // Calculate standard deviation
  const squaredDiffs = sortedTimes.map(time => Math.pow(time - avg, 2));
  const avgSquaredDiff = squaredDiffs.reduce((sum, diff) => sum + diff, 0) / squaredDiffs.length;
  const stdDev = Math.sqrt(avgSquaredDiff);

  return {
    avg: avg,
    p95: p95,
    min: Math.min(...sortedTimes),
    max: Math.max(...sortedTimes),
    stdDev: stdDev,
    totalExecutions: sortedTimes.length
  };
}

/**
 * Main performance test function
 */
export function runPerformanceTest() {
  console.log('Starting Search Performance Baseline Test...');

  // Measure baseline performance
  const baselineTime = measureBaselinePerformance();
  console.log(`Baseline performance (no search): ${baselineTime.toFixed(2)}ms for 1000 operations`);

  // Measure search performance
  const searchTimes = measureSearchPerformance();
  console.log(`Completed ${searchTimes.length} search operations`);

  // Calculate statistics
  const stats = calculateStats(searchTimes);

  console.log('\n=== Performance Results ===');
  console.log(`Average search time: ${stats.avg.toFixed(2)}ms`);
  console.log(`P95 search time: ${stats.p95.toFixed(2)}ms`);
  console.log(`Min search time: ${stats.min.toFixed(2)}ms`);
  console.log(`Max search time: ${stats.max.toFixed(2)}ms`);
  console.log(`Standard deviation: ${stats.stdDev.toFixed(2)}ms`);

  // Validate against requirements
  console.log('\n=== Requirement Validation ===');

  // Requirement: <500ms search update (SC-002)
  const meetsResponseTime = stats.avg < 500;
  console.log(`✓ <500ms response time: ${meetsResponseTime ? 'PASS' : 'FAIL'} (${stats.avg.toFixed(2)}ms)`);

  // Requirement: <200ms p95 (Plan: <200ms p95)
  const meetsP95 = stats.p95 < 200;
  console.log(`✓ <200ms p95 time: ${meetsP95 ? 'PASS' : 'FAIL'} (${stats.p95.toFixed(2)}ms)`);

  // Calculate performance degradation
  const avgSearchTime = stats.avg;
  const operationsCount = 1000; // Using same count as baseline for comparison
  const baselinePerOperation = baselineTime / operationsCount;
  const searchPerOperation = avgSearchTime / operationsCount;

  const degradationPercent = ((searchPerOperation - baselinePerOperation) / baselinePerOperation) * 100;
  const meetsDegradation = degradationPercent < 10;

  console.log(`✓ <10% performance degradation: ${meetsDegradation ? 'PASS' : 'FAIL'} (${degradationPercent.toFixed(2)}%)`);

  // Overall result
  const overallPass = meetsResponseTime && meetsP95 && meetsDegradation;
  console.log(`\n=== Overall Result: ${overallPass ? 'PASS' : 'FAIL'} ===`);

  return {
    stats,
    requirements: {
      responseTime: meetsResponseTime,
      p95Time: meetsP95,
      degradation: meetsDegradation
    },
    overallPass
  };
}

/**
 * Run the performance test when this module is executed directly
 */
if (typeof window !== 'undefined' && window.document) {
  // Browser environment
  window.runSearchPerformanceTest = runPerformanceTest;
} else if (typeof module !== 'undefined' && module.exports) {
  // Node.js environment
  module.exports = { runPerformanceTest };
}

console.log('Performance baseline testing functions loaded.');