pipeline {
    agent {
        docker {
         image 'golang'
        }
    }

    stages {
        stage('Dependencies') {
            steps {
                sh 'go get ./...'
            }
        }
        stage('Build') {
            steps {
                sh 'go build ./...'
            }
        }
        stage('Test') {
            steps {
                sh 'go test ./...'
            }
        }
    }
}
