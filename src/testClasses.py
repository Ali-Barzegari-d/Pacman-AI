import inspect
import re
import sys
class Question(object):

    def raiseNotDefined(self):
        print('Method not implemented: %s' % inspect.stack()[1][3])
        sys.exit(1)

    def __init__(self, questionDict, display):
        self.maxPoints = int(questionDict['max_points'])
        self.testCases = []
        self.display = display

    def getDisplay(self):
        return self.display

    def getMaxPoints(self):
        return self.maxPoints

    def addTestCase(self, testCase, thunk):
        self.testCases.append((testCase, thunk))

    def execute(self, grades):
        self.raiseNotDefined()

class PassAllTestsQuestion(Question):

    def execute(self, grades):
        testsFailed = False
        grades.assignZeroCredit()
        for _, f in self.testCases:
            if not f(grades):
                testsFailed = True
        if testsFailed:
            grades.fail("Tests failed.")
        else:
            grades.assignFullCredit()

class ExtraCreditPassAllTestsQuestion(Question):
    def __init__(self, questionDict, display):
        Question.__init__(self, questionDict, display)
        self.extraPoints = int(questionDict['extra_points'])

    def execute(self, grades):

        testsFailed = False
        grades.assignZeroCredit()
        for _, f in self.testCases:
            if not f(grades):
                testsFailed = True
        if testsFailed:
            grades.fail("Tests failed.")
        else:
            grades.assignFullCredit()
            grades.addPoints(self.extraPoints)


class HackedPartialCreditQuestion(Question):

    def execute(self, grades):
        grades.assignZeroCredit()

        points = 0
        passed = True
        for testCase, f in self.testCases:
            testResult = f(grades)
            if "points" in testCase.testDict:
                if testResult: points += float(testCase.testDict["points"])
            else:
                passed = passed and testResult

        ## FIXME: Below terrible hack to match q3's logic
        if int(points) == self.maxPoints and not passed:
            grades.assignZeroCredit()
        else:
            grades.addPoints(int(points))
class Q6PartialCreditQuestion(Question):

    def execute(self, grades):
        grades.assignZeroCredit()

        results = []
        for _, f in self.testCases:
            results.append(f(grades))
        if False in results:
            grades.assignZeroCredit()

class PartialCreditQuestion(Question):

    def execute(self, grades):
        grades.assignZeroCredit()

        for _, f in self.testCases:
            if not f(grades):
                grades.assignZeroCredit()
                grades.fail("Tests failed.")
                return False
class NumberPassedQuestion(Question):
    """Grade is the number of test cases passed."""

    def execute(self, grades):
        grades.addPoints([f(grades) for _, f in self.testCases].count(True))

class TestCase(object):

    def raiseNotDefined(self):
        print('Method not implemented: %s' % inspect.stack()[1][3])
        sys.exit(1)

    def getPath(self):
        return self.path

    def __init__(self, question, testDict):
        self.question = question
        self.testDict = testDict
        self.path = testDict['path']
        self.messages = []

    def __str__(self):
        self.raiseNotDefined()

    def execute(self, grades, moduleDict, solutionDict):
        self.raiseNotDefined()

    def writeSolution(self, moduleDict, filePath):
        self.raiseNotDefined()
        return True

    def testPass(self, grades):
        grades.addMessage('PASS: %s' % (self.path,))
        for line in self.messages:
            grades.addMessage('    %s' % (line,))
        return True

    def testFail(self, grades):
        grades.addMessage('FAIL: %s' % (self.path,))
        for line in self.messages:
            grades.addMessage('    %s' % (line,))
        return False

    def testPartial(self, grades, points, maxPoints):
        grades.addPoints(points)
        extraCredit = max(0, points - maxPoints)
        regularCredit = points - extraCredit

        grades.addMessage('%s: %s (%s of %s points)' % ("PASS" if points >= maxPoints else "FAIL", self.path, regularCredit, maxPoints))
        if extraCredit > 0:
            grades.addMessage('EXTRA CREDIT: %s points' % (extraCredit,))

        for line in self.messages:
            grades.addMessage('    %s' % (line,))
        return True
    def addMessage(self, message):
        self.messages.extend(message.split('\n'))

