# Data Model: ROS 2 Education Content Structure

## Entities

### Chapter
- **id**: string (unique identifier for the chapter)
- **title**: string (display title of the chapter)
- **description**: string (brief description of the chapter content)
- **tags**: array[string] (keywords for search and categorization)
- **prerequisites**: array[string] (knowledge required before reading)
- **learning_objectives**: array[string] (what the learner will understand after reading)
- **content**: string (path to the markdown file)
- **module_id**: string (reference to the parent module)

### Module
- **id**: string (unique identifier for the module)
- **title**: string (display title of the module)
- **description**: string (overview of the module content)
- **chapters**: array[Chapter] (list of chapters in the module)
- **estimated_duration**: number (time in minutes to complete the module)

### Course
- **id**: string (unique identifier for the course)
- **title**: string (main title of the course)
- **description**: string (overall course description)
- **modules**: array[Module] (list of modules in the course)
- **target_audience**: string (who the course is designed for)
- **prerequisites**: array[string] (baseline knowledge required)

## Relationships
- Course contains many Modules
- Module contains many Chapters
- Chapter belongs to one Module
- Module belongs to one Course

## Validation Rules
- Each Chapter must have a unique id within its Module
- Each Module must have a unique id within its Course
- Chapter titles must be 3-6 words for clarity
- Learning objectives must be specific and measurable
- Prerequisites must reference other content in the system

## State Transitions
- Content draft → reviewed → published → archived
- Each state transition requires specific validation and approval