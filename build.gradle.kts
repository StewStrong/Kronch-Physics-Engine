plugins {
    kotlin("jvm") version "1.4.21"
    id("org.jlleitschuh.gradle.ktlint") version "10.0.0"
    java
    maven
    checkstyle
    id("org.sonarqube") version "3.1.1"
}

group = "org.valkyrienskies.kronch"
version = "1.0"

repositories {
    mavenCentral()
    maven { setUrl("https://dl.bintray.com/kotlin/kotlin-eap") }
}

dependencies {
    // Kotlin
    implementation(kotlin("stdlib-jdk8"))

    // JOML for Math
    api("org.joml:joml:1.10.0")
    api("org.joml:joml-primitives:1.10.0")

    // Guava
    implementation("com.google.guava:guava:29.0-jre")

    // FastUtil for Fast Primitive Collections
    implementation("it.unimi.dsi", "fastutil", "8.2.1")

    // Junit 5 for Unit Testing
    testImplementation("org.junit.jupiter:junit-jupiter:5.4.2")
}

tasks.withType<Checkstyle> {
    reports {
        // Do not output html reports
        html.isEnabled = false
        // Output xml reports
        xml.isEnabled = true
    }
}

checkstyle {
    toolVersion = "8.41"
    configFile = file("$rootDir/.checkstyle/checkstyle.xml")
    isIgnoreFailures = false
}

ktlint {
    disabledRules.set(setOf("parameter-list-wrapping"))
    reporters {
        reporter(org.jlleitschuh.gradle.ktlint.reporter.ReporterType.CHECKSTYLE)
    }
}

tasks {
    compileKotlin {
        kotlinOptions.jvmTarget = "1.8"
    }
    compileTestKotlin {
        kotlinOptions.jvmTarget = "1.8"
    }
    compileJava {
        sourceCompatibility = "1.8"
        targetCompatibility = "1.8"
    }
    compileTestJava {
        sourceCompatibility = "1.8"
        targetCompatibility = "1.8"
    }
    test {
        useJUnitPlatform()
        testLogging {
            events("passed", "skipped", "failed")
        }
    }
}

sonarqube {
    properties {
        property("sonar.projectKey", "ValkyrienSkies_kronch")
        property("sonar.organization", "valkyrienskies")
        property("sonar.host.url", "https://sonarcloud.io")
    }
}
