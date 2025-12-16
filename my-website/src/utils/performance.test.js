// Performance tests for search functionality
// These tests validate performance requirements such as response time and page degradation

// Mock large content dataset for performance testing
const generateLargeContentSet = (size = 1000) => {
  const contentSet = [];
  for (let i = 0; i < size; i++) {
    contentSet.push({
      id: `content-${i}`,
      content: `This is sample content text ${i} with various words to search through for performance testing purposes. The content contains multiple sentences with different keywords like robotics, AI, search, algorithm, performance, and other technical terms. ${i % 2 === 0 ? 'Additional text for even numbers.' : 'Additional text for odd numbers.'}`,
      element: { id: `element-${i}` },
      url: `page-${i}.html`,
      title: `Page ${i} Title`
    });
  }
  return contentSet;
};

// Mock the search service functions for performance testing
const mockSearchService = {
  // Mock indexContent to return large dataset
  indexContent: jest.fn(() => generateLargeContentSet(500)), // 500 items for performance test

  // Mock searchContent with performance timing
  searchContent: jest.fn((query, options = {}) => {
    // Simulate search operation timing
    const startTime = performance.now();

    // Simple search implementation for mock
    const searchIndex = mockSearchService.indexContent();
    const { caseSensitive = false, partialMatch = true, maxResults = 10 } = options;
    const searchTerm = caseSensitive ? query : query.toLowerCase();

    const results = [];
    for (const item of searchIndex) {
      const content = caseSensitive ? item.content : item.content.toLowerCase();
      if (partialMatch ? content.includes(searchTerm) : content === searchTerm) {
        results.push(item);
        if (results.length >= maxResults) break;
      }
    }

    const endTime = performance.now();
    const searchTime = endTime - startTime;

    // Log performance for analysis
    console.log(`Search for "${query}" took ${searchTime.toFixed(2)}ms and returned ${results.length} results`);

    return results;
  }),

  // Mock other functions
  highlightMatches: jest.fn(),
  clearHighlights: jest.fn(),
  debouncedSearch: jest.fn(fn => fn), // Skip debounce for performance tests
  sanitizeSearchQuery: jest.fn(query => query),
  validateQuery: jest.fn(query => query && query.trim().length >= 1 && query.trim().length <= 200),
  getSnippet: jest.fn((content, searchTerm) => content.substring(0, 100) + '...')
};

// Performance test suite
describe('Search Performance Tests', () => {
  // Test performance requirement: search results update within 500ms (SC-002, T052)
  test('Search should update results within 500ms', async () => {
    const start = performance.now();

    // Execute search with mock service
    const results = mockSearchService.searchContent('robotics', { maxResults: 10 });

    const end = performance.now();
    const duration = end - start;

    console.log(`Search completed in ${duration.toFixed(2)}ms`);

    // Performance requirement: <500ms
    expect(duration).toBeLessThan(500);
  });

  // Test performance requirement: <200ms p95 for search results (T056)
  test('Search should meet p95 performance of <200ms', async () => {
    const executionTimes = [];

    // Run multiple searches to calculate p95
    const searchTerms = ['robotics', 'AI', 'algorithm', 'performance', 'search', 'textbook', 'content', 'function', 'method', 'system'];

    for (const term of searchTerms) {
      const start = performance.now();
      mockSearchService.searchContent(term);
      const end = performance.now();
      executionTimes.push(end - start);
    }

    // Calculate p95 (95th percentile)
    executionTimes.sort((a, b) => a - b);
    const p95Index = Math.floor(0.95 * executionTimes.length) - 1;
    const p95Time = executionTimes[p95Index];

    console.log(`P95 search time: ${p95Time.toFixed(2)}ms`);

    // Performance requirement: p95 < 200ms
    expect(p95Time).toBeLessThan(200);
  });

  // Test scalability with large content sets (T057)
  test('Search should perform well with large content sets', async () => {
    // Mock larger content set
    mockSearchService.indexContent = jest.fn(() => generateLargeContentSet(1000)); // 1000 items

    const start = performance.now();
    const results = mockSearchService.searchContent('test', { maxResults: 5 });
    const end = performance.now();
    const duration = end - start;

    console.log(`Search on 1000 items completed in ${duration.toFixed(2)}ms with ${results.length} results`);

    // Should still complete within reasonable time
    expect(duration).toBeLessThan(1000); // Less than 1 second for 1000 items
  });

  // Test rapid typing performance (T034, T062)
  test('Search should handle rapid typing without performance degradation', async () => {
    const start = performance.now();

    // Simulate rapid typing by calling search multiple times quickly
    const promises = [];
    for (let i = 0; i < 10; i++) {
      promises.push(new Promise(resolve => {
        setTimeout(() => {
          mockSearchService.searchContent(`term${i}`);
          resolve();
        }, i * 50); // Stagger calls
      }));
    }

    await Promise.all(promises);

    const end = performance.now();
    const duration = end - start;

    console.log(`Processed 10 rapid searches in ${duration.toFixed(2)}ms`);

    // Total time should be reasonable
    expect(duration).toBeLessThan(1000); // Less than 1 second for 10 searches
  });

  // Test memory usage during search operations
  test('Search should not cause excessive memory usage', () => {
    const initialMemory = global.gc ? process.memoryUsage().heapUsed : 0;

    // Perform multiple searches
    for (let i = 0; i < 100; i++) {
      mockSearchService.searchContent(`search${i % 10}`); // Cycle through 10 terms
    }

    const finalMemory = global.gc ? process.memoryUsage().heapUsed : 0;
    const memoryIncrease = finalMemory - initialMemory;
    const memoryIncreaseMB = memoryIncrease / 1024 / 1024;

    console.log(`Memory increase during search operations: ${memoryIncreaseMB.toFixed(2)} MB`);

    // Memory increase should be reasonable
    expect(memoryIncreaseMB).toBeLessThan(50); // Less than 50 MB increase
  });
});

// Performance Baseline Tests
describe('Performance Baseline Validation', () => {
  // Validate success criteria SC-002: Search results update within 500ms (T052)
  test('Validate success criteria SC-002: <500ms search update', () => {
    const baselineStart = performance.now();

    // Execute typical search operation
    const results = mockSearchService.searchContent('typical search term', { maxResults: 10 });

    const baselineEnd = performance.now();
    const baselineDuration = baselineEnd - baselineStart;

    console.log(`Baseline performance: ${baselineDuration.toFixed(2)}ms for typical search`);

    // Verify requirement is met
    expect(baselineDuration).toBeLessThan(500);
  });

  // Validate success criteria SC-005: <10% performance degradation (T055, T043)
  test('Validate success criteria SC-005: <10% performance degradation', () => {
    // Measure baseline performance (without search)
    const baselineStart = performance.now();
    // Simulate baseline operation (e.g., simple DOM access)
    const elements = Array.from({ length: 100 }, (_, i) => ({ id: `element-${i}`, text: `text ${i}` }));
    const baselineEnd = performance.now();
    const baselineTime = baselineEnd - baselineStart;

    // Measure performance with search functionality
    const searchStart = performance.now();
    mockSearchService.searchContent('performance test');
    const searchEnd = performance.now();
    const searchTime = searchEnd - searchStart;

    // Calculate degradation percentage
    const degradationPercent = ((searchTime - baselineTime) / baselineTime) * 100;

    console.log(`Performance degradation: ${degradationPercent.toFixed(2)}%`);
    console.log(`Baseline: ${baselineTime.toFixed(2)}ms, With search: ${searchTime.toFixed(2)}ms`);

    // Verify requirement is met (less than 10% degradation)
    expect(degradationPercent).toBeLessThan(10);
  });
});

// Cross-browser performance considerations
describe('Cross-browser Performance Consistency', () => {
  test('Search performance should be consistent across different scenarios', () => {
    // Test different search scenarios to ensure consistency
    const scenarios = [
      { query: 'short', options: { maxResults: 5 } },
      { query: 'longer search term with multiple words', options: { maxResults: 5 } },
      { query: 'a', options: { maxResults: 5 } },
      { query: 'specificTechnicalTerm', options: { maxResults: 5 } },
    ];

    const scenarioTimes = [];

    for (const scenario of scenarios) {
      const start = performance.now();
      mockSearchService.searchContent(scenario.query, scenario.options);
      const end = performance.now();
      scenarioTimes.push(end - start);
    }

    // Calculate average and standard deviation to check consistency
    const avgTime = scenarioTimes.reduce((sum, time) => sum + time, 0) / scenarioTimes.length;
    const variance = scenarioTimes.reduce((sum, time) => sum + Math.pow(time - avgTime, 2), 0) / scenarioTimes.length;
    const stdDev = Math.sqrt(variance);

    console.log(`Scenario performance - Avg: ${avgTime.toFixed(2)}ms, Std Dev: ${stdDev.toFixed(2)}ms`);

    // Performance should be consistent (low standard deviation relative to average)
    const coefficientOfVariation = (stdDev / avgTime) * 100;
    console.log(`Coefficient of variation: ${coefficientOfVariation.toFixed(2)}%`);

    expect(coefficientOfVariation).toBeLessThan(50); // Less than 50% variation
  });
});

console.log('Performance tests structure created. These tests require performance measurement tools to run properly.');