pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                go build ./...
            }
        }
        stage('Test') {
            steps {
                go test ./...
            }
        }
    }
}
